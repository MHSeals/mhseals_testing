"""The loop: ticks, keys and focus; it renders only on a change."""

import errno

from mhseals_testing.runner import fold
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.ui import keymap
from mhseals_testing.ui.layout import ROW
from mhseals_testing.ui import perf
from mhseals_testing.ui.popups import pins

from mhseals_testing.ui.app.shared import CURSORS, Detach, STOPPING_ERRORS
from mhseals_testing.ui.app.help import HELP, help_rows, help_width


NEIGHBOURS = {   # a name or the first of a tuple of names that is on screen
    True: {'status': {'right': 'motion'},
           'motion': {'left': 'status', 'right': 'conditions', 'down': 'procedures'},
           'conditions': {'left': 'motion', 'right': 'minimap', 'down': 'thrusters'},
           'thrusters': {'left': 'motion', 'right': 'minimap', 'up': 'conditions', 'down': 'activity'},
           'minimap': {'left': 'thrusters', 'down': 'activity'},
           'procedures': {'left': 'status', 'right': 'activity', 'up': 'motion'},
           'activity': {'left': 'procedures', 'up': 'thrusters'}},
    False: {'status': {'right': ('motion', 'conditions')},
            'motion': {'left': 'status', 'down': 'conditions'},
            'conditions': {'left': 'status', 'up': 'motion', 'down': 'thrusters'},
            'thrusters': {'left': 'status', 'up': 'conditions', 'down': 'procedures'},
            'procedures': {'left': 'status', 'up': 'thrusters', 'down': 'activity'},
            'activity': {'left': 'status', 'up': 'procedures'}},
}


FOCUS = {'focus_left': 'left', 'focus_right': 'right', 'focus_up': 'up', 'focus_down': 'down'}   # global
SHIFTED = ('take_control', 'operator_state', 'release_hold')   # global Shift letters, never the procedure ones
PANEL_SCOPES = {'thrusters': 'panel.thrusters', 'activity': 'panel.activity', 'status': 'panel.status'}  # win over Drive


def terminal_gone(error):
    """True for the key reader's EOF or EIO: the terminal is gone, not a stopping error."""
    return isinstance(error, EOFError) or error.errno == errno.EIO


class Loop:
    """The loop, mixed into App."""

    def tick(self):
        if getattr(self, '_runner_paused', False):
            self.poll_pause()
            return
        if self.review:
            self.tick_review()
            if self._review_live is not None and self._review_live['source'].live:
                replay = self.source
                try:
                    self.source = self._review_live['source']
                    self.tick_live()
                finally:
                    self.source = replay
            if self.args.review is None and self.armed and self.arming.fault():
                self.stop(self.arming.fault())  # review over a live session still stops on a fault
            return
        self.tick_live()

    def tick_review(self):
        """Advance the replay; reader, decode and ingest data errors end only the review."""
        if getattr(self, '_review_fault', False):
            return
        try:
            self.review.tick(self.clock(), self.clock() - self.last_tick)
            self.last_tick = self.clock()
            self.source.tick(self.clock())
        except (RuntimeError, OSError, ValueError, KeyError) as error:
            self.review.pause()
            self.review.due = None
            message = f'Review playback failed: {error}'
            if self.args.review is None and self._review_live is not None:
                self.leave_review()          # live owners and recording were never touched
            else:
                self._review_fault = True    # standalone replay: stay paused, stop reading
            self.notice = message

    def tick_live(self):
        """Keep the existing live owner work running while review only replaces its view."""
        self.source.tick(self.clock())
        if not self.source.live:
            return  # replay: no host reads, config watching or driving
        self.refresh_advisories()
        self.watch_config()
        if self.feeds:
            snapshot = self.snapshot()
            for feed in self.feeds:
                feed.tick(self, snapshot, self.clock())
        if not self.watching:
            self.fence_alert()
        if self.nav2_file is not None:
            self.notice = self.nav2_file.poll() or self.notice
        if self.test_pulses is not None and self.publish_pulses is not None:
            self.publish_pulses(self.test_pulses)
        self.pulse_readback()
        if self.armed and self.arming.fault():
            self.stop(self.arming.fault(), by='fault')
        self.watch_watchdogs()
        perf.tick(self)

    def run(self):
        perf.start(self)        # config/perf.yaml: rows-only output in place of Rich's Live, and the Meter
        with self.attached():
            try:
                self.offer_restore()
                while not self.quit:
                    try:
                        key = self.keys.read(timeout=0.1)
                        if key is not None:
                            self.perf.key()
                            try:
                                self.dispatch(key)
                            except STOPPING_ERRORS as error:
                                self.stop(str(error) or type(error).__name__, by='error')
                            perf.draw(self)              # the key's frame first, before the tick's work
                        self.tick()
                        perf.draw(self, force=False)     # idle ticks render only on change
                    except KeyboardInterrupt as error:
                        self.detaching = isinstance(error, Detach)
                        # Ctrl+C quits like q, minus the question: anything moving has
                        # already stopped at neutral (the runner neutralizes on any exit).
                        self.quit = True
                        self.modal = None
                        self.notice = 'Interrupted'
                    except (EOFError, OSError) as error:
                        if not terminal_gone(error):
                            raise
                        # docker exec never hangs up a dead client's pty; quit like q
                        # instead of lingering headless with the recorder and outputs.
                        self.quit = True
                        self.modal = None
                        self.notice = 'Terminal closed'
            finally:
                self.close()  # on q and on Ctrl+C alike, while the page is still up

    def focus_on(self, panel):
        if self.focus == 'thrusters' and panel != 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.focus = panel

    def show_tab(self, name):
        """Switch to the tab ``name`` (focus and cursors are restored by TabMemory)."""
        step = self.tabs.tabs.index(name) - self.tabs.tabs.index(self.tab)
        if step:
            self.switch_tab(step)

    def on_minimap(self, key, keys=None):
        """view.minimap keys (Drive column and Map tab): zoom, and the Layers box toggles."""
        action = (keys or self.map_keys).get(key)
        if action in ('zoom_in', 'zoom_out'):
            self.minimap.zoom(1 if action == 'zoom_in' else -1)
        elif action == 'zoom_fit':
            self.minimap.reset()
        else:
            self.layerbox.press(action, self.minimap.half)

    def on_map(self, key):
        """The Map tab adds tab.map's toggles, whose keys are Drive procedures elsewhere."""
        self.on_minimap(key, self.map_tab_keys)

    def switch_tab(self, step):
        """Move to the next or previous tab, restoring the focus and cursors it was left with."""
        if self.focus == 'thrusters':
            self.clear_pulses()  # test pulses only while their panel has focus
        self.grabbed = None
        self.focus, saved = self.tabs.switch(step, self.focus, self.cursors)
        self.cursors = {**dict.fromkeys(CURSORS, 0), **saved}

    def move(self, step):
        count = self.rows(self.focus)
        if count:
            self.cursors[self.focus] = max(0, min(count - 1, self.cursors[self.focus] + step))

    def dispatch(self, key):
        """Apply one key on the page."""
        if self.review:
            folded = fold(key)
            if self.dispatch_review(folded) or not self.review_allows(folded):
                return
        if not self.review and self.annotate_key(key):   # b N " and an offered pick digit; never Space or E
            return
        checklist = getattr(self, 'checklist', None)
        if checklist is not None and not self.review and checklist.handle(self, key):
            return
        bound = keymap.dispatch('global', key)          # case kept (K1)
        if bound == 'camera' and not self.review:  # Shift-V; action 'v' still toggles Results/log on Activity
            self.perception_popup(view=True)  # ui.md keymap camera: View already open
            return
        if bound == 'history':  # Shift-H opens the history everywhere; action 'h' still adjusts a setting
            self.open('history')
            return
        if bound == 'labels' and not self.review:   # the Labels tab: review this session's marks and segments; L again leaves it
            if self.tab == 'Labels':
                self.show_tab(self.labels.came_from or self.tabs.tabs[0])
            else:
                self.labels.came_from = self.tab
                self.show_tab('Labels')
            return
        if bound == 'back' and self.tab == 'Labels' and not self.review and not self.modal:   # Esc leaves Labels too
            self.show_tab(self.labels.came_from or self.tabs.tabs[0])
            return
        if key == keymap.REVOKE_KEY:   # fixed: no keymap file moves it
            self.revoke_agent()
            return
        if bound in SHIFTED:
            {'take_control': self.take_control, 'operator_state': self.operator_menu,
             'release_hold': self.release_hold}[bound]()
            return
        raw, key = key, fold(key)
        if keymap.matches('panel.thrusters', 'pins', raw) and self.focus == 'thrusters' and not self.review:  # action 'm' stays Manual
            self.notice = pins.edit_pins(self)
            return
        if keymap.matches('panel.thrusters', 'pin', raw) and self.focus == 'thrusters' and not self.review:  # p stays Perception
            self.notice = pins.edit_row_pin(self, self.cursors['thrusters'])
            return
        if (keymap.matches('panel.activity', 'exclude', raw) and self.focus == 'activity' and not self.review
                and self.activity == 'results'):
            self.exclude_result()  # panel-level Exclude run; action 'x' still writes the report
            return
        action = keymap.dispatch('global', key)          # Shift folds to the lowercase binding
        if action == 'odometry':  # the Pose popup is global
            self.open('odometry')
            return
        if bound == 'health' and self.engine is not None:
            self.open('health')
            return
        if self.tab == 'Labels' and not self.review and self.on_labels(raw):
            return
        if self.tab == 'Correction' and not self.review and self.on_correction(raw):
            return
        if self.tab == 'Fusion' and not self.review and self.fusion_keys.get(raw) == 'camera_image':
            self.fusion_image = not self.fusion_on()
            self.notice = f"camera image {'on' if self.fusion_image else 'off'}"
            return
        panels = self.panels()
        if action in ('tab_prev', 'tab_next'):
            self.switch_tab(-1 if action == 'tab_prev' else 1)
        elif action in ('focus_next', 'focus_prev'):
            index = panels.index(self.focus) if self.focus in panels else 0
            self.focus_on(panels[(index + (-1 if action == 'focus_prev' else 1)) % len(panels)])
        elif action in FOCUS:
            wanted = NEIGHBOURS[self.console.size.width >= ROW].get(self.focus, {}).get(FOCUS[action], ())
            neighbour = next((p for p in ((wanted,) if isinstance(wanted, str) else wanted) if p in panels), None)
            if neighbour:
                self.focus_on(neighbour)
        elif action in ('up', 'down'):
            step = -1 if action == 'up' else 1
            if self.focus == 'thrusters' and self.grabbed is not None and not self.review:
                self.move_grabbed(step)
            else:
                self.move(step)
        elif key == keymap.STOP_KEY:   # fixed
            self.operator_stop()
        elif action == 'quit':
            self.confirm_quit()
        elif action == 'back':  # backs out; never quits
            if self.grabbed is not None:
                self.grabbed = None
                self.notice = 'Dropped'
        elif action == 'help':
            self.show('Help', rows=help_rows(self.profile, HELP), width=help_width(getattr(getattr(self, 'console', None), 'width', None)))
        elif action == 'arm':
            self.arm_panel()
        elif bound == 'report':
            if not self.toasts.ack_newest(self.monitor.clock()):   # x dismisses a toast first
                self.export()
        elif bound == 'mark_note':
            self.mark_note()
        elif action in ('amplitude_up', 'amplitude_down'):
            step = 0.05 if action == 'amplitude_up' else -0.05
            self.set_setting('amplitude', round(min(1.0, max(0.05, self.settings.amplitude + step)), 2))
            self.notice = f'Amplitude {self.settings.amplitude:g} {self.profile.command_units}'
        elif (key in {p.key for p in PROCEDURES} and raw == key and self.tab == 'Drive'  # Shift is never a procedure
              and not (self.focus in PANEL_SCOPES and keymap.dispatch(PANEL_SCOPES[self.focus], key))  # thrusters s,
              and not (self.focus == 'minimap' and key in self.map_keys)):  # activity v, status e; view.minimap e centres
            self.on_procedures(key)
        elif self.focus == 'list':
            self.on_list(raw)
        elif self.focus in ('minimap', 'map'):
            getattr(self, f'on_{self.focus}')(raw)
        elif raw == key:  # a Shift letter has no panel binding yet; folding it hit the lowercase ones
            getattr(self, f'on_{self.focus}', lambda key: None)(key)
