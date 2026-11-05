"""The Status panel rows: Enter and left/right on a row, the Agents setting and the popup openers."""

from mhseals_testing.nav2 import edit as nav2_edit
from mhseals_testing.ui import parts, popups
from mhseals_testing.ui.feeds import agent as agent_feed
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import course, stop as stop_popup


class StatusRows:
    """The Status panel rows, mixed into Settings."""

    def on_status(self, key):
        rows = status_rows(self.snapshot(), self.profile.sensors)
        if key in ('left', 'right') and self.agents_row(rows):
            self.toggle_agents()
        elif key in ('left', 'right') and self.row_setting(rows) == 'on_exit' and not self.watching:
            state = self.map_state()
            client = self.fence_client()
            if state is not None and client is not None:
                self.notice = course.toggle_on_exit(self, client, state.fence_status)
        elif key in ('left', 'right') and self.row_setting(rows) == 'agent_motion':
            agent_feed.step_setting(self, self.row_setting(rows), key)
        elif key in ('left', 'right') and self.row_setting(rows) == 'recording':
            self.step_recording(key)
        elif key in ('left', 'right') and self.row_setting(rows) == 'graphics':
            self.step_graphics(key)
        elif key == 'e':
            row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
            if row and row['name'] == nav2_edit.NAME and self.nav2_file is not None:
                self.nav2_popup()
        elif key == 'enter':
            if rows:
                row = rows[min(self.cursors['status'], len(rows) - 1)]
                if row.get('source') == 'sensor' and row['name'] == 'odometry':
                    return self.open('odometry')
                if row.get('source') == 'part' and not self.watching:
                    return parts.toggle(self, row['key'])
                if row.get('source') == 'stack' and not self.watching:
                    return parts.toggle_stack(self)
                if row['name'] == 'Driving':
                    return self.open('authority')
                if row['name'] == 'Course area' and not self.watching:
                    self.notice = course.open_course(self)
                    return None
                if row['name'] == 'Hold':
                    return stop_popup.open_stop(self) if row.get('stop') else self.release_hold()
                if row['name'] in ('RTK', 'Source'):
                    return self.open('odometry')
                if row['name'] == 'Config':
                    return self.open('config')
                if row['name'].lower() == 'battery':   # the health advisory is named 'battery'
                    return self.open('power')
                return popups.inspect.open_inspect(self, row)   # the view fits the row (sensor, tf, ...)

    def row_setting(self, rows):
        row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
        return row.get('setting') if row else None

    def agents_row(self, rows):
        return self.row_setting(rows) == 'agents'

    def toggle_agents(self):
        """Flip the Agents setting; a bound arbiter follows (block refuses new agent claims)."""
        self.settings.agents = 'block' if self.settings.agents == 'allow' else 'allow'
        arbiter = getattr(self, 'arbiter', None)
        if arbiter is not None:
            arbiter.allow_agents = self.settings.agents == 'allow'
        self.notice = f'Agents {self.settings.agents}'
        desk = getattr(self, 'desk', None)
        if desk is not None:
            desk.setting_changed()           # block stops an agent run in flight, as E does

    def open(self, action_id):
        """Open the popup registered for ``action_id`` in ``popups.OPENERS``; an unbound id does nothing."""
        opener = popups.OPENERS.get(action_id)
        if opener is None:
            return False
        opener(self)
        return True

    def on_conditions(self, key):
        if key == 'enter':
            self.open('conditions')
