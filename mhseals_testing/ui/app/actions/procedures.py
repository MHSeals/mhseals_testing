"""Procedures: running one, its context, pausing, and its run record."""

from contextlib import contextmanager
from mhseals_testing.model import Cancelled
from mhseals_testing.health.config import config_path
from mhseals_testing.strict_yaml import load_strict
from mhseals_testing.nav2 import limits as nav2_limits
from mhseals_testing.harness import host, operator_state, runs, session as session_module
from mhseals_testing.conditions import STILL_FRESH_S, stillness
from mhseals_testing.runner import Aborted, Runner, STALE_MUX_S, fold, override_cause
from mhseals_testing.procedures import PROCEDURES, unavailable
from mhseals_testing.readiness import motion_blockers
from mhseals_testing.ros.arming import muxed
from mhseals_testing.ui import agent_acts, keymap as keymap_module, layout, progress, theme
from mhseals_testing.ui.feeds.agent_desk import desk as agent_feed
from mhseals_testing.ui.popups import magcal as magcal_popup


def run_end(error):
    """How a procedure run that raised ``error`` ended, for its run record: ``outcome``
    (aborted, cancelled or error) and ``cause`` (the abort's cause, or what failed). A
    KeyboardInterrupt (Ctrl-C) aborts the run with cause ``interrupted``."""
    if isinstance(error, KeyboardInterrupt):
        return dict(outcome='aborted', cause='interrupted', by='operator')
    if isinstance(error, Aborted):
        return dict(outcome='aborted', cause=error.cause, by=error.by)
    if isinstance(error, Cancelled):
        return dict(outcome='cancelled', cause=str(error) or 'cancelled')
    return dict(outcome='error', cause=f'{type(error).__name__}: {error}')


def guard_end(cause, stopped_by):
    """``(reason, ended)`` for a run a watchdog ended (``cause``: the fence's ``geofence``), whoever stopped it."""
    return f'{cause}: the {cause} watchdog stopped the run', dict(outcome='aborted', cause=cause, by='guard',
                                                                   stopped_by=stopped_by)


def nav2_params():
    """mhseals_nav's checked-in Nav2 parameters, or None outside a ROS install."""
    try:
        path = config_path('config', 'nav2_params.yaml', package='mhseals_nav', installed=True)
    except LookupError:
        return None
    return path if path.exists() else None


def agent_run(app):
    """``{who, claim}`` of the agent act whose run this is, or None (an operator's run)."""
    act = agent_feed.run_act(app)
    return None if act is None else dict(who=act['who'], claim=act['claim'])

class Procedures:
    """Procedures, mixed into App."""

    def moving(self):
        return bool(self.state.procedures.running or self.services.manual.active)

    def characterize(self, direction):
        """r: re-refine ``direction`` (the cursor column); R: the whole survey. The setting is this run's only."""
        procedure = next(p for p in PROCEDURES if p.key == 'f')
        before, self.settings.direction = self.settings.direction, direction
        try:
            self.run_procedure(procedure)
        finally:
            self.settings.direction = before

    def on_motion(self, key):
        pass

    def on_procedures(self, key):
        if keymap_module.matches('global', 'activate', key):
            self.run_procedure(PROCEDURES[self.cursors['procedures']])
            return
        procedure = next((p for p in PROCEDURES if p.key == key), None)
        if procedure is not None:
            self.cursors['procedures'] = PROCEDURES.index(procedure)
            self.run_procedure(procedure)

    def availability(self, procedure):
        if self.watching:
            return f'watching pid {self.watching}: no commands'
        reason = unavailable(procedure, self.profile, self.armed, self.args.bench_override)
        # Procedures that measure motion can't be picked without odometry and TF, nor
        # while another node commands the same topic.
        if not reason and 'sensors' in procedure.needs and not self.args.bench_override:
            blockers = self.motion_blockers() if self.armed else motion_blockers(
                self.snapshot(), self.profile)
            if blockers:
                reason = blockers[0]
        if not reason and self.engine is not None:
            blocking, _ = self.engine.gate(
                procedure.key, self.args.bench_override, 'armed' in procedure.needs)
            reason = blocking[0] if blocking else ''
        return reason

    def run_procedure(self, procedure):
        if self.review:
            self.say(f'{procedure.title}: procedures are off in review')
            return
        if not self.source.live:
            self.say('Procedures are disabled in review')
            return
        reason = self.availability(procedure)
        if reason and 'armed' in procedure.needs and not self.armed:
            self.arm_panel()  # offer to arm, then carry on with the procedure
            reason = self.availability(procedure)
        if reason:
            self.say(f'{procedure.title}: {reason}')
            return
        self.clear_pulses()
        desk = self.desk
        if desk is not None and procedure.key == 'm':
            desk.take_over()             # manual cancels an agent goal at once, as Space and E do
        if procedure.key == 'm' and not operator_state.paused(operator_state.load()):
            self.set_operator_state('manual', reason='manual control (m)', notice=False)
        try:
            # Only jump to Map once it has a body; otherwise the operator lands on a placeholder.
            if (self.settings.auto_tab and 'Map' in layout.BODIES
                    and procedure.group in ('Navigate', 'Mission')):
                self.show_tab('Map')
            run = self.state.procedures
            run.running = procedure.title
            results = len(self.snapshot()['results'])
            run.run_id = runs.next_run_id(self.paths.runs, procedure.title)
            run.run_fields = {}   # run-level fields a procedure adds to its run record (mission)
            run.tags = dict(procedure=procedure.title, key=procedure.key, run_id=run.run_id)
            self.say(f'Running {procedure.title}')
            self.health_snapshot('start', procedure)
            if procedure.key == 'f':           # characterization (f, and an agent's run f): the fence follows
                self.anchor_fence()
            began = self.engine.clock() if self.engine else 0.0
        except BaseException:              # manual never outlives the drive it was set for
            self.state.procedures.running, self.state.procedures.tags = None, {}
            self.end_manual(procedure)
            raise
        aborted, ended = None, dict(outcome='complete', cause=None)
        try:
            try:
                procedure.run(self.procedure_context())
            except BaseException:
                self.health_window(procedure, began)   # before teardown, whatever the exit
                raise
            invalid = self.health_window(procedure, began)
            stopped = next((r.get('verdict') for r in self.snapshot()['results'][results:]
                            if r.get('verdict') in runs.ABORT_VERDICTS), None)
            if stopped:      # its goal ended early (the fence's return goal preempts it): aborted, never complete
                guard = self.guard_cause()
                aborted, ended = guard_end(guard, 'guard') if guard else (
                    f'{procedure.title} {stopped.lower()}', dict(outcome='aborted', cause=stopped.lower(),
                                                                 by='procedure'))
        except BaseException as error:    # a KeyboardInterrupt is recorded too, then re-raised
            aborted, ended = str(error), run_end(error)
            guard = self.guard_cause() if isinstance(error, Aborted) else ''
            if guard and ended.get('cause') != guard:  # the stop came while the watchdog held: its cause, not a cancel
                aborted, ended = guard_end(guard, ended.get('by'))
            if not isinstance(error, Cancelled):
                raise
            self.say(f'{procedure.title} cancelled')
            self.disarm(f'{procedure.title} cancelled')
            return
        finally:
            self.state.procedures.running, self.state.procedures.tags = None, {}
            self.end_manual(procedure)
            self.write_run(procedure, results, aborted, ended)
        if len(self.snapshot()['results']) > results:  # show what it found
            self.focus, self.activity = 'activity', 'results'
            self.cursors['activity'] = 0
            if aborted is not None:
                self.say(f"{procedure.title} ABORTED: {ended.get('cause')}")
            elif not invalid:
                self.say(f'{procedure.title} complete')

    def write_run(self, procedure, start, aborted, ended=None):
        """The one run file for a procedure run: every result it recorded since ``start``, how the
        run ended (``ended``: outcome and cause, see run_end) and the procedure's run fields, which
        win (a mission's ``refused``). A run with neither results, an abort nor fields leaves none."""
        found = self.snapshot()['results'][start:]
        run = self.state.procedures
        run_id, run.run_id = run.run_id, None
        fields = run.run_fields or {}
        if not found and aborted is None and not fields:
            return
        key = agent_acts.procedure(procedure.key)[1]   # its agent name (mission); title: display
        record = dict(ended or {}, **fields, procedure=procedure.title, run_key=key,
                      run_id=run_id, results=found)
        if aborted is not None:
            record['reason'] = aborted
        runs.write_run(self.paths.runs, record, settings=vars(self.settings),
                       status=runs.ABORTED if aborted is not None else None,
                       provenance=session_module.provenance(
                           self.facts, self.history, self.args.profile))

    def health_window(self, procedure, began):
        """Snapshot health and flag the trial window invalid when a gated check left ok.

        Returns the invalid notice (also shown), or None.
        """
        self.health_snapshot('end', procedure)
        if self.engine is None:
            return None
        problems = self.journal.window_problems(
            self.engine, procedure.key, began, self.engine.clock(), 'armed' in procedure.needs)
        if problems:
            self.ros.emit('trial_window_invalid', procedure=procedure.key, reasons=problems)
            self.say(f'{procedure.title}: window invalid, {problems[0]}')
            return self.notice
        return None

    def procedure_context(self):
        act = agent_feed.run_act(self) or {}   # an agent's run: the desk's act names its task and nav profile
        return host.context(
            self, self.profile, self.monitor, self.settings, self.args.stale_after,
            dict(
                conditions=self.conditions, nav2=self.nav2, task_watcher=self.task_watcher,
                session_dir=self.paths.directory, start=self.processes.start,
                running=self.processes.running, stop=self.processes.stop,
                returncode=self.processes.returncode,
                nav2_params=nav2_params(), nav2_overlay=self.paths.nav2,
                nav2_user=nav2_limits.path(),
                hardware_config=self.args.hardware_config, wall_ns=self.wall_ns,
                user_config=self.args.config,
                history=self.history, warn=self.warn_history,
                staging=self.staging(), fit_context=self.fit_context, ask_text=self.ask_text,
                driver_status=lambda: self.ros.driver_status,
                disarm=self.disarm, pulses=self.set_pulses, save_hardware=self.save_hardware,
                mux_input=self.mux_input,
                magcal=lambda: magcal_popup.ask_heading(self), type_word=self.type_word,
                task=act.get('task'),     # mission: the task an agent's run named
                agent=agent_run(self),    # an agent's run: {who, claim}; its save questions answer no
                reset_to_start=self.reset_to_start,   # mission: sim-only, gated by mission.sim_stack
                mission_profile=act.get('profile'),   # and its nav profile (finals)
                run_id=self.state.procedures.run_id, run_fields=self.state.procedures.run_fields,
                manual=dict(plant=self.plant, blockers=self.manual_now, checks=self.manual_checks,
                            open_events=self.open_events, show=self.manual_show, emit=self.emit,
                            speed=lambda: self.services.manual.speed, clock=self.monitor.clock,
                            annotate=self.annotate_later,
                            options=self.manual_options)))

    def motion(self, name, **details):
        """A Runner that commands the plant, inside the live page."""
        return self.session(name, self.motion_blockers, details)

    def supervise(self, name, **details):
        """A Runner for watching Nav2 drive; the suite does not command meanwhile."""
        return self.session(name, self.supervision_blockers, details)

    def observe(self, name, **details):
        """A passive measurement window; no arming or readiness needed."""
        return self.session(name, lambda: [], details)

    def resume_gate(self):
        """Fresh odometry, independent of the neutral-water estimator's selected source."""
        if self.armed and muxed(self.profile) and self.arbiter is None:
            return ['authority is unknown']
        row = self.monitor.snapshot()['sensors'].get('odometry', {})
        age = row.get('age', float('inf'))
        if (row.get('status') != 'LIVE' or row.get('issue')
                or not 0 <= age <= STILL_FRESH_S):
            return ['pose is missing, stale or invalid']
        stamp_age = row.get('stamp_age')
        if stamp_age is not None and not -.25 <= stamp_age <= STILL_FRESH_S:
            return ['pose timestamp is invalid']
        values = row.get('values', {})
        return stillness(values.get('vx'), values.get('vy'), values.get('wz'))

    def cancel_run(self, cause):
        """Abort the run in flight at its next key or paused tick; the first cause names it."""
        run = self.state.procedures
        run.cancelled = run.cancelled or cause

    def poll_pause(self):
        """Refresh read-only sources and views; never replay pulses or watch/apply config."""
        self.source.tick(self.clock())
        arbiter = self.arbiter
        if arbiter is not None:
            arbiter.tick()
        snapshot = self.snapshot()
        for feed in self.feeds:
            feed.tick(self, snapshot, self.clock())

    def runner_key(self, key, keymap):
        if self.annotate_key(key):     # marks during a run (never Space, E or abort)
            return None
        action = keymap.resolve(['runner'], key)
        if action is None and keymap.resolve(['runner'], fold(key)) == 'abort':
            action = 'abort'  # Caps Lock must not defeat abort; E/U remain case-sensitive.
        if action == 'abort':
            return 'abort'
        if action is None and keymap.resolve(['tab.drive'], key) == 'manual':
            # A manual takeover: the run (an agent's too) ends at neutral; m again drives.
            desk = self.desk
            if desk is not None:
                desk.cancel('overridden: keyboard')
            return 'abort'
        if action in ('mark', 'revoke_agent', 'release_hold'):
            {'mark': self.mark, 'revoke_agent': self.revoke_agent,
             'release_hold': self.release_hold}[action]()
            if self.state.procedures.cancelled:
                return 'abort'

    @contextmanager
    def session(self, name, blockers, details):
        def show(command, remaining, label):
            self.state.procedures.paused = False
            self.set_command(command)
            self.monitor.command(command)
            place = runner.place
            if place is not None:     # a long run (characterization): bar, step, command against measured
                self.say(progress.line(place, command, self.measured_velocity(), remaining, theme.SEPARATOR)
                         + f'{theme.SEPARATOR}Space aborts')
            else:
                self.say(f'{label or name}{theme.SEPARATOR}{remaining:.1f}s{theme.SEPARATOR}Space aborts')
            if self.services.manual.test_pulses is not None and self.publish_pulses is not None:
                self.publish_pulses(self.services.manual.test_pulses)
            self.refresh(force=False)

        self.state.procedures.cancelled = False
        keymap, _ = keymap_module.load()

        def paused_tick(cause, elapsed):
            run = self.state.procedures
            run.paused = True
            if run.cancelled:      # name what ended it and what paused it (a fence hold is no operator)
                ended = run.cancelled if isinstance(run.cancelled, str) else 'by operator'
                raise Aborted(f'motion cancelled {ended} while paused by {cause}')
            self.poll_pause()
            self.set_command()
            self.monitor.command(self.services.manual.command)
            self.say(f'PAUSED{theme.SEPARATOR}{cause}{theme.SEPARATOR}{elapsed:.1f}s'
                     f'{theme.SEPARATOR}U release · E revoke · Space aborts')
            self.refresh(force=False)

        def on_pause():     # the human drives with the deployed C, the survey with identity
            if self.staging().active:
                self.staging().suspend()

        def on_resume():
            if self.staging().active:
                self.staging().resume()

        runner = Runner(self.plant, lambda: blockers() + agent_feed.blockers(self), self.keys, show=show,
                        emit=self.emit, clock=self.monitor.clock,
                        override=self.override_cause, gate=self.resume_gate,
                        paused_tick=paused_tick, key_action=lambda key: self.runner_key(key, keymap),
                        on_pause=on_pause, on_resume=on_resume)
        try:
            with runner.session(name, **details):
                yield runner
        finally:
            self.state.procedures.paused = False
            self.set_command()

    def measured_velocity(self):
        """(vx, vy, wz) from live odometry, or None."""
        row = self.monitor.snapshot()['sensors'].get('odometry', {})
        values = row.get('values') or {}
        found = [values.get(key) for key in ('vx', 'vy', 'wz')]
        return found if row.get('status') == 'LIVE' and all(isinstance(v, (int, float)) for v in found) else None

    def override_cause(self):
        """Why the command mux is not under automation (derive's rules in harness/authority.py).

        '' when no arbiter is bound or the profile has no mux; when disarmed
        (the mux only runs as an arming child) a missing or stale state is '' too.
        """
        arbiter = self.arbiter
        if arbiter is None or not muxed(self.profile):
            return ''
        with arbiter.lock:
            mux, at, pause = arbiter.mux, arbiter.mux_at, arbiter.pause_request
        age = float('inf') if at is None else arbiter.clock() - at
        if not self.armed and (mux is None or age > STALE_MUX_S):
            return ''
        return override_cause(mux, age, pause)

    def reset_to_start(self, timeout):
        """crane_ml's /crane/reset_to_start; mission.reset_to_start calls it on a sim stack only."""
        run = self.state.procedures
        if run.start_reset is None:
            from mhseals_testing.procedures.mission import START_RESET
            from mhseals_testing.ros.sim_reset import StartReset
            run.start_reset = StartReset(self.ros, load_strict(START_RESET.read_text(), START_RESET)['service'])
        return run.start_reset(timeout)

    def task_watcher(self):
        if self.state.procedures.watcher is None:
            if self.watcher_factory is None:
                raise RuntimeError('no task watcher in this session')
            self.state.procedures.watcher = self.watcher_factory()
        return self.state.procedures.watcher

    def nav2(self):
        if self.state.procedures.nav2 is None:
            if self.nav2_factory is None:
                raise RuntimeError('no Nav2 client in this session')
            self.state.procedures.nav2 = self.nav2_factory()
        return self.state.procedures.nav2
