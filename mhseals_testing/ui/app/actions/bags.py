"""Recording and bags: segments, the bag index, the Bags tab and bag review."""

import threading

from mhseals_testing.model import Cancelled
from mhseals_testing import data
from mhseals_testing.harness import bags as bags_module, index, journal
from mhseals_testing.harness.bags import MODES as REC_MODES
from mhseals_testing.harness.recwatch import DISK_MARKER
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import feeds as feeds_module, keymap, theme
from mhseals_testing.ui.panels import activity, agent_log, scrubber
from mhseals_testing.ui.layout import TabMemory
from mhseals_testing.ui.minimap import panel as minimap_panel
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.popups import run as run_popup
from mhseals_testing.ui.review import BIG, SMALL, load_session, review_metadata
from mhseals_testing.ui.source import ReviewSource

from mhseals_testing.ui.app.shared import CURSORS
from mhseals_testing.ui.app.help import UPDOWN


SEEKS = {'seek_back': (SMALL, -1), 'seek_fwd': (SMALL, 1), 'seek_back_big': (BIG, -1), 'seek_fwd_big': (BIG, 1)}
LIST = {'activate': 'open_bag_review', 'pin': 'pin_bag', 'title': 'title_bag', 'delete': 'delete_bag',
        'prune': 'prune_bags', 'finalize': 'finalize_bag'}            # tab.bags (Enter: global activate)
REVIEW_REFUSED = ('arm', 'report', 'amplitude_up', 'amplitude_down', 'manual')   # act on the boat
REVIEW_ALLOWED = ('focus_next', 'focus_prev', 'focus_left', 'focus_right', 'focus_up', 'focus_down',
                  'up', 'down', 'help', 'odometry')                  # global keys that reach the page


class Bags:
    """Recording and bags, mixed into App."""

    def watch_recording(self):
        """Sample the recorder's health; below the free-disk floor stop it, mark it and say so."""
        self.rec = self.recwatch.sample(self.paths.bag, self.processes.running('bag'),
                                       self.recorder.mode)
        if self.rec.stop_for_disk:
            self.ros.emit('marker', kind='system', note=DISK_MARKER)   # first, so it lands in the bag
            self.set_recording('off', reason='disk')
            self.say(f'{DISK_MARKER} ({self.rec.free_gb:.1f} GB free)')

    def set_recording(self, mode, reason='operator'):
        """Recording row: end the running segment (``rec_stop``), start the next unless ``off`` (``rec_start``)."""
        with self.recording_lock:
            if mode != 'off' and self.paths.directory.resolve() == self.finalizing:
                self.say('Recording refused while this session is being finalized')
                return
            self._set_recording(mode, reason)

    def _set_recording(self, mode, reason):
        if self.recorder.segment is not None:
            self.ros.emit('rec_stop', segment=self.recorder.segment.name, reason=reason)
        segment = self.recorder.set_mode(mode)
        self.ros.recording = segment is not None  # events stamp ``rec`` from this (bags.md 1)
        self.rebuild_index()                      # INDEX.json at each segment close
        if segment is not None:
            self.ros.emit('rec_start', segment=segment.name, mode=mode)
        self.say(f'Recording {mode}' + (f' {segment.name}' if segment else ''))

    def rebuild_index(self):
        """Rewrite INDEX.json and README.md; a failure is logged to the notice, never fatal."""
        sessions = self.paths.directory.parent
        if self.watching or self.args.review is not None or sessions.name != 'sessions':
            return    # an explicit --output outside a data root has no index to keep
        try:
            index.rebuild(sessions.parent)
        except OSError as error:
            self.say(f'Index not rebuilt: {error}')

    def step_recording(self, key):
        """left/right on the Recording row moves along lean, full, off without wrapping."""
        i = REC_MODES.index(self.recorder.mode) + (1 if keymap.matches('panel.status', 'adjust_up', key) else -1)
        if 0 <= i < len(REC_MODES):
            self.set_recording(REC_MODES[i])

    def recording_paths(self):
        """Authoritative live session and exact writer segment, resolved at action time."""
        if not self.processes.running('bag'):
            return None, None
        segment = self.recorder.segment
        return self.paths.directory.resolve(), segment.resolve() if segment is not None else None

    def bag_protected(self, bag):
        session, segment = self.recording_paths()
        path = bag.path.resolve()
        return path == session or (segment is not None and (path == segment or path in segment.parents))

    def invalidate_bags(self):
        self.bag_scan = (float('-inf'), *self.bag_scan[1:])

    def bags_view(self):
        """One lazy cache; writer transitions invalidate even within the five-second interval."""
        now = self.clock()
        recording_key = self.recording_paths()
        if recording_key != self.bag_recording_key or now - self.bag_scan[0] >= 5:
            recording = {recording_key[0].name} if recording_key[0] is not None else ()
            try:
                self.bag_scan = (now, bags_module.scan(recording=recording), bags_module.disk())
            except OSError:
                self.bag_scan = (now, [], (0, 0, 0))
            self.bag_recording_key = recording_key
            self.cursors['list'] = max(0, min(self.cursors['list'], len(self.bag_scan[1]) - 1))
        return self.bag_scan[1], self.bag_scan[2]

    def selected_bag(self):
        found = self.bags_view()[0]
        if not found:
            self.say('No sessions')
            return None
        return found[self.cursors['list']]

    def check_bag(self, bag, destructive=False, metadata=None):
        """Revalidate live paths and metadata; cached row flags never authorize a mutation."""
        if self.bag_protected(bag):
            raise PermissionError(f'{bag.id} is recording')
        if bag.path.resolve() == getattr(self, 'finalizing', None):
            raise PermissionError(f'{bag.id} is being finalized')
        header = metadata if metadata is not None else data.read_json(bag.meta_path, {})
        bag.pinned = bool(header.get('pinned'))
        if destructive and bag.pinned:
            raise PermissionError(f'{bag.id} is pinned')

    def refresh_bag_index(self, bag):
        root = bag.path.parent if bag.legacy else bag.path.parent.parent
        try:
            index.rebuild(root)
        finally:
            self.invalidate_bags()

    def on_list(self, key):
        action = keymap.dispatch(('global', 'tab.bags'), key)
        handler = getattr(self, LIST[action]) if action in LIST else None
        if handler is None:
            return
        bag = None if action == 'prune' else self.selected_bag()
        if action != 'prune' and bag is None:
            return
        try:
            handler(bag)
        except Cancelled:
            pass
        except (OSError, ValueError) as error:
            self.say(f'{key}: {error}')

    def pin_bag(self, bag):
        self.check_bag(bag)
        bags_module.set_meta(bag, pinned=not bag.pinned)
        self.refresh_bag_index(bag)
        self.say(f'{"Pinned" if bag.pinned else "Unpinned"} {bag.id}')

    def title_bag(self, bag):
        self.check_bag(bag)
        title = self.ask_text('Title', f'Title for {bag.id}', bag.title)
        if title is not None:
            self.check_bag(bag)
            bags_module.set_meta(bag, title=title.strip())
            self.refresh_bag_index(bag)
            self.say(f'Titled {bag.id}')

    def delete_bag(self, bag):
        self.check_bag(bag, destructive=True)
        if self.choose(f'Delete {bag.id}?', ('Delete', 'Cancel'), {'y': 0}, default=1,
                       danger=True, message='Session files are removed for good.') == 'Delete':
            self.check_bag(bag, destructive=True)
            try:
                bags_module.delete(bag)
            finally:
                self.refresh_bag_index(bag)
            self.say(f'Deleted {bag.id}')

    def prune_bags(self, _bag):
        eligible = []
        for bag in self.bags_view()[0]:
            try:
                self.check_bag(bag, destructive=True)
            except PermissionError:
                continue
            eligible.append(bag)
        # Keep protected sessions in the total budget, but never among candidates.
        protected = [b for b in self.bags_view()[0] if b not in eligible]
        budget = bags_module.budget_bytes() - sum(b.size for b in protected)
        doomed = bags_module.prune_candidates(eligible, budget)
        if not doomed:
            self.say('Nothing to prune')
            return
        if self.choose(f'Prune {len(doomed)} sessions?', ('Delete', 'Cancel'), {'y': 0},
                       default=1, danger=True, rows=[(b.id, '') for b in doomed]) != 'Delete':
            return
        attempted = False
        try:
            for bag in doomed:
                self.check_bag(bag, destructive=True)
                attempted = True
                bags_module.delete(bag)
            self.say(f'Pruned {len(doomed)} sessions')
        finally:
            if attempted:
                self.refresh_bag_index(doomed[0])

    def finalize_bag(self, bag):
        """Claim one session before starting the quiet worker; recording cannot enter it."""
        with self.recording_lock:
            if self.armed or self.moving():
                self.say('Finalize refused while armed or moving')
                return
            if self.finalizing is not None:
                self.say('Finalize is still running')
                return
            self.check_bag(bag)
            segments = sorted({f.parent for f in bag.path.rglob('*.mcap')
                               if not (f.parent / 'metadata.yaml').exists()})
            if not segments:
                self.say(f'{bag.id}: nothing to finalize')
                return
            self.finalizing = bag.path.resolve()
            self.say(f'Finalize {bag.id}: {len(segments)} segments in the background')
            self.finalize_thread = threading.Thread(target=self._finalize, args=(bag, segments), daemon=True)
            try:
                self.finalize_thread.start()
            except Exception:
                self.finalizing = None
                raise

    @staticmethod
    def segment_stamp(segment):
        """Detect cleanup even if reindex fails after removing an empty MCAP."""
        paths = [*segment.glob('*.mcap'), segment / 'metadata.yaml']
        return [(p.name, p.stat().st_size, p.stat().st_mtime_ns) for p in paths if p.exists()]

    def _finalize(self, bag, segments):
        changed, results = False, []
        try:
            for segment in segments:
                if self.bag_protected(bag):
                    results.append('refused')
                    break
                before = self.segment_stamp(segment)
                try:
                    # Read the actual segment at execution time, never paths.bag.
                    result = bags_module.finalize(segment, self.quiet_run, self.recording_paths()[1])
                    results.append(result['status'])
                finally:
                    changed |= before != self.segment_stamp(segment)
            self.say(f'Finalize {bag.id}: ' + ', '.join(
           f'{results.count(r)} {r}' for r in dict.fromkeys(results)))
        except Exception as error:
            self.say(f'Finalize {bag.id}: {error}')
        finally:
            try:
                if changed:
                    self.refresh_bag_index(bag)
            except Exception as error:
                self.say(f'Finalize {bag.id}: index update failed: {error}')
            finally:
                with self.recording_lock:
                    self.finalizing = None

    def mark_note(self, number=None):
        """Type a note for marker ``number`` (default the last); never while moving."""
        found = journal.markers(journal.read(self.paths.journal))
        if number is None:
            number = found[-1]['id'] if found else None
        if number is None:
            self.say('No marker yet; b adds one')
        elif self.moving():
            self.say('Not while moving')
        else:
            note = self.text_input(f'Marker {number}', 'Note for the marker (optional)')
            if note:
                self.ros.emit('marker_note', id=number, note=note)

    def dispatch_review(self, key):
        """Review's own keys (context global -> review). True when the key was consumed.

        Keys that would act on the boat (arm, procedures, export) are refused here, so the
        live paths never see them; focus, help and quit fall through to the page.
        """
        review, now, action = self.review, self.clock(), keymap.dispatch('review', key)
        if action in SEEKS:
            seconds, sign = SEEKS[action]
            review.seek(sign * seconds, now)
        elif action in ('jump_prev', 'jump_next'):
            review.jump(-1 if action == 'jump_prev' else 1, now)
        elif action in ('rate_down', 'rate_up'):
            review.faster(-1 if action == 'rate_down' else 1)
        elif keymap.matches('review.activity', 'zoom', key) and self.focus == 'activity':
            review.loupe = scrubber.next_loupe(review.loupe)
        elif action == 'play':
            review.toggle()
        elif key == keymap.STOP_KEY:
            self.operator_stop()  # Keep the global sticky hold and neutral/disarm safety.
            review.pause()
        elif action == 'leave':
            self.leave_review()
        elif keymap.matches('global', 'quit', key):
            if self.args.review is not None:
                self.quit = True        # nothing of ours to lose, no popup
            else:
                self.confirm_quit()
        elif keymap.matches('panel.activity', 'view_next', key) and self.focus == 'activity':
            return False            # the view switch is read-only
        elif key in {p.key for p in PROCEDURES} or keymap.dispatch(('global', 'tab.drive'), key) in REVIEW_REFUSED:
            self.say('Review: arming, procedures and edits are off')
        else:
            return False
        return True

    def review_allows(self, key):
        """Whitelist for keys that reach the page in review: focus, cursor, help and read-only details."""
        return (keymap.dispatch('global', key) in REVIEW_ALLOWED
                or (keymap.matches('global', 'activate', key) and self.focus == 'activity')
                or (keymap.matches('panel.activity', 'view_next', key) and self.focus == 'activity'))

    def open_bag_review(self, bag):
        """Open the selected closed session without replacing live owners or callbacks."""
        if self.review:
            return
        if self.armed or self.services.manual.active or self.services.procedures.running or self.services.manual.test_pulses is not None:
            self.say('Review: disarm and stop motion or test pulses first')
            return
        header = review_metadata(bag.meta_path)
        self.check_bag(bag, metadata=header)
        selected = load_session(bag.path)
        name = header.get('profile')
        if isinstance(name, dict):
            name = name.get('name')
        profile = load_profile(name) if name else self.profile
        selected.host = header.get('host', selected.host)
        selected.title = header.get('title') or bag.id
        try:
            replay = ReviewSource(bag.path, profile, selected)
        except (ImportError, RuntimeError) as error:
            raise ValueError(f'cannot read session: {error}') from error
        self._review_live = {'source': self.source, 'tabs': self.tabs, 'focus': self.focus,
                             'cursors': self.cursors, 'activity': self.activity,
                             'minimap': self.minimap, 'layerbox': self.layerbox}
        self.rebuild(source=replay, review=selected, tabs=TabMemory(),
                     minimap=minimap_panel.MiniMap(self.map_state), layerbox=LayerBox(None, self.map_state, live=False))
        self._review_fault = False
        selected.load = replay.seek
        self.focus, self.cursors = 'activity', dict.fromkeys(CURSORS, 0)
        self.activity = 'log'
        self.restart_ticks()
        self.say(f'Review {bag.id}; Esc returns to live')

    def review_failed(self, error):
        """Playback hit a data error: a review over live returns to live; a standalone one stays paused, unread."""
        self.review.pause()
        self.review.due = None
        message = f'Review playback failed: {error}'
        if self.args.review is None and self._review_live is not None:
            self.leave_review()          # live owners and recording were never touched
        else:
            self._review_fault = True
        self.say(message)

    def leave_review(self):
        """Back to live; a session opened with --review has no live to return to, so it quits."""
        if self.args.review is not None:
            self.quit = True
        else:
            live, self._review_live = self._review_live or {}, None
            self.focus, self.cursors, self.activity = (live.pop(name, getattr(self, name))
                                                       for name in ('focus', 'cursors', 'activity'))
            self.rebuild(review=None, **live)
            self.restart_ticks()
            self.say('Left review')

    def on_activity(self, key):
        action = keymap.dispatch(('global', 'panel.activity'), key)
        if action == 'view_next':
            desk = feeds_module.merged(getattr(self, 'feeds', ())).get('agent_desk')
            self.activity, self.cursors['activity'] = agent_log.next_view(self.activity, desk), 0
        elif action == 'activate' and self.activity == 'results':
            self.open_run()
        elif action == 'activate' and self.activity == 'agent':
            agent_log.detail(self, feeds_module.merged(self.feeds).get('agent_desk'), self.cursors['activity'])

    def exclude_result(self, index=None):
        """Ask for a reason and exclude the result at oldest-first ``index`` (default: the cursor's)."""
        total = len(self.snapshot()['results'])
        if not total:
            return
        if index is None:
            index = total - 1 - min(self.cursors['activity'], total - 1)
        try:
            reason = self.choose('Exclude run', self.EXCLUDE_REASONS)
        except Cancelled:
            return
        if reason:
            self.monitor.exclude_result(index, run_popup.exclusion(reason))

    def open_run(self):
        """popup.run for the cursor's result: up/down step, X excludes, Esc closes; Space stops."""
        total = len(self.snapshot()['results'])
        if not total:
            return
        # Pinned by oldest-first index so a result landing mid-popup cannot change the run shown.
        state = dict(index=total - 1 - min(self.cursors['activity'], total - 1))

        def fill():
            result = self.snapshot()['results'][state['index']]
            modal.update(title=activity.result_label(result), rows=run_popup.run_rows(result),
                         hint=theme.SEPARATOR.join((f'{UPDOWN} prev/next', 'X exclude', 'Esc close')))

        def handle(key):
            if keymap.matches('popup.run', 'exclude', key):  # case matters here: lowercase x writes the report
                self.exclude_result(state['index'])
                self.modal = modal  # the nested popup cleared it
            else:
                action = keymap.dispatch('popup.run', key, fold=True)
                if key == keymap.STOP_KEY:
                    self.stop('stopped by operator')
                elif action in ('prev', 'next'):
                    last = len(self.snapshot()['results']) - 1
                    state['index'] = min(max(state['index'] + (1 if action == 'prev' else -1),
                                             0), last)
                    self.cursors['activity'] = last - state['index']
                elif action == 'close':
                    return True
            fill()

        modal = dict(kind='message', width=76)
        fill()
        self.popup(modal, handle)
