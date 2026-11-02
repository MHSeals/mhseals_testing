"""Quitting: the confirm popup and closing every output, child and file in order."""

from contextlib import contextmanager

from mhseals_hardware.menu import Cancelled
from mhseals_testing import calibration
from mhseals_testing.harness import journal, session as session_module
from mhseals_testing.report import build_report, write_report
from mhseals_testing.ui import parts, theme
from mhseals_testing.ui import imaging as pixels


class Quit:
    """Quitting, mixed into App."""

    def export(self):
        report = build_report(self.profile, self.snapshot(), self.paths.bag,
                              self.args.hardware_config, journal_events=journal.read(self.paths.journal),
                              conditions=self.conditions.summary())
        path = write_report(report, self.paths.report)
        self.notice = f'Report saved: {path}'
        return path

    def sync_guests(self):
        """Emit guest_attach / guest_detach as boat_trial guests join and leave the registry."""
        if self.registry is None or self.watching:
            return
        guests = self.registry.state().get('guests', {})
        for pid in sorted(set(guests) - self.guests_seen):
            self.ros.emit('guest_attach', pid=pid, **guests[pid])
        for pid in sorted(self.guests_seen - set(guests)):
            self.ros.emit('guest_detach', pid=pid)
        self.guests_seen = set(guests)

    @contextmanager
    def attached(self):
        """One key reader and one full-screen live view for the whole session."""
        # No refresh thread: the loop draws right after each key, and nothing else writes
        # to the terminal between frames, so mode switches such as Kitty's reach it intact.
        # The image tier is probed on the key reader's fd before the screen switches (ui/imaging/);
        # graphics: off (the default) probes nothing and draws the text as before.
        with self.open_keys() as keys:
            pixels.forget()
            pixels.start(keys, self.console, self.write_terminal, theme.PALETTE.graphics)
            try:
                with self.live_factory(self.render(), console=self.console, screen=True,
                                       auto_refresh=False) as live:
                    self.keys, self.live = keys, live
                    try:
                        yield self
                    finally:
                        self.keys = self.live = None
            finally:
                pixels.stop(self.console, self.write_terminal)

    def confirm_quit(self):
        """Say what quitting does, and offer to keep an unsaved mapping."""
        rows = [('Outputs', 'neutral, then disarm' if self.armed else 'already disarmed'),
                ('Recording', f'flush {self.paths.bag}'),
                ('Report', str(self.paths.report)),
                ('Results', str(len(self.snapshot()['results'])))]
        options = ['Quit', 'Cancel']
        if self.draft is not None:
            rows.append(('Mapping', 'unsaved changes'))
            if self.pins_moved():   # pins save only through s, which reads them back
                rows.append(('Pins', 'unsaved pin change is dropped'))
            options = ['Save mapping and quit', 'Quit without saving', 'Cancel']
        remote = [] if self.watching else parts.quit_rows(self)
        rows += remote        # asked once; Quit stops only what this TUI started
        if any(value.endswith('Quit stops them') for _, value in remote):
            options.insert(-1, 'Quit, keep remote parts running')
        if any(value.endswith('Quit keeps them running') for _, value in remote):
            options.insert(-1, 'Quit, also stop adopted parts')
        try:
            choice = self.choose('Quit', options, {'y': 0, 'n': len(options) - 1}, rows=rows,
                                 default=len(options) - 1, width=72)
        except Cancelled:
            return
        if choice == 'Cancel':
            return
        self.keep_units = choice == 'Quit, keep remote parts running'
        self.stop_adopted = choice == 'Quit, also stop adopted parts'
        if choice == 'Save mapping and quit':
            channel_map, reversed_ = self.mapping()
            if not self.save_hardware(channel_map=channel_map, reversed=reversed_):
                return
        self.quit = True

    def close(self):
        """Shut down in order, showing each step: nothing may drive, then flush, then report.

        Safe to call more than once; main() calls it too in case the page never opened.
        """
        if self.closed:
            return
        self.closed = True
        self.health.close()
        if self.args.review is not None:
            return  # nothing of ours to flush; the session's report must not be rewritten
        steps = [('Outputs', self.close_outputs), ('Children', self.close_children),
                 ('Remote parts', self.close_units), ('Report', self.close_report), ('Tracking', self.close_tracking),
                 ('Calibration', self.close_calibration),
                 ('Recording', self.close_recording), ('Index', self.close_index)]
        if self.detaching:   # the children and the recording go on under the next owner
            steps = [('Outputs', self.close_outputs), ('Handover', self.close_handover)]
        if self.watching:    # not ours to stop: neutral, events and the report belong to the owner
            steps = [('Watch', lambda: f'left pid {self.watching} running')]
        rows = [[name, theme.icon('busy').plain + ' waiting'] for name, _ in steps]
        self.modal = dict(kind='message', title='Closing session', rows=rows, width=64,
                          hint='Ctrl+C again skips the rest')
        for row, (_, step) in zip(rows, steps):
            row[1] = theme.icon('busy').plain + ' working'
            self.refresh()
            try:
                row[1] = theme.icon('ok').plain + ' ' + step()
            except Exception as error:  # keep going: later steps still matter
                row[1] = f"{theme.icon('bad').plain} {error}"
        self.refresh()
        self.modal = None

    def close_outputs(self):
        was_armed = self.armed
        self.stop('session ended')
        return 'neutral and disarmed' if was_armed else 'were disarmed'

    def close_children(self):
        names = [name for name in list(self.processes.processes)
                 if name != 'bag'] if hasattr(self.processes, 'processes') else []
        for name in names:
            self.processes.stop(name)
        return f"stopped {', '.join(names)}" if names else 'none running'

    def close_units(self):
        """Stop the units we started on other computers (Quit's default), or leave them to their
        lease; adopted ones are stopped only on the operator's explicit choice."""
        units = getattr(self.stack, 'units', None)
        return units.close(stop=not self.keep_units, adopted=self.stop_adopted) \
            if units is not None else 'none'

    def close_report(self):
        self.ros.emit('session_stop')
        return self.export().name

    def close_tracking(self):
        verdicts = self.tracking.close()
        return ', '.join(f"{a} {'stale' if v['flag'] else 'ok'}" for a, v in verdicts.items()) or 'no scores'

    def close_calibration(self):
        wrote = calibration.write_snapshot(self.args.hardware_config)
        return 'status written' if wrote else 'nothing fitted'

    def close_recording(self):
        if not self.processes.running('bag'):
            return 'was not running'
        self.set_recording('off', reason='quit')  # SIGINT lets ros2 bag finish the file
        return f'flushed {self.paths.directory.name}'

    def close_index(self):
        session_module.finish(self.paths.directory)
        self.rebuild_index()
        return 'INDEX.json rebuilt'

    def summary(self):
        """What the session left behind, printed once the page is gone (and ROS is closed, so it reads only
        the source: the live snapshot's Stack rows ask ROS for its nodes)."""
        results = self.source.snapshot()['results']
        passed = sum(1 for r in results if theme.kind(r.get('verdict', '')) == 'good')
        minutes = (self.clock() - self.started) / 60
        return [('Duration', f'{minutes:.1f} min'), ('Results', f'{passed}/{len(results)} good'),
                ('Bag', str(self.paths.bag)), ('Report', str(self.paths.report))]
