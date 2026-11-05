"""The Status panel rows: Enter and left/right on a row, the Agents setting and the popup openers."""

from mhseals_testing.nav2 import edit as nav2_edit
from mhseals_testing.ui import keymap, parts, popups
from mhseals_testing.ui.feeds.agent_desk import desk as agent_feed
from mhseals_testing.ui.panels.status import status_rows
from mhseals_testing.ui.popups import course, stop as stop_popup


# keymap panel.status adjust_down / adjust_up -> the direction the row step helpers read
STEPS = {'adjust_down': 'left', 'adjust_up': 'right'}


class StatusRows:
    """The Status panel rows, mixed into Settings."""

    def on_status(self, key):
        """Status panel keys (keymap panel.status; Enter is global activate)."""
        rows = status_rows(self.snapshot(), self.profile.sensors)
        action = keymap.dispatch('panel.status', key)
        step = STEPS.get(action)
        if step and self.agents_row(rows):
            self.toggle_agents()
        elif step and self.row_setting(rows) == 'on_exit' and not self.watching:
            state = self.map_state()
            client = self.fence_client()
            if state is not None and client is not None:
                self.say(course.toggle_on_exit(self, client, state.fence_status))
        elif step and self.row_setting(rows) == 'agent_motion':
            agent_feed.step_setting(self, self.row_setting(rows), step)
        elif step and self.row_setting(rows) == 'recording':
            self.step_recording(key)                    # it asks panel.status itself
        elif step and self.row_setting(rows) == 'graphics':
            self.step_graphics(step)
        elif action == 'params':
            row = rows[min(self.cursors['status'], len(rows) - 1)] if rows else None
            if row and row['name'] == nav2_edit.NAME and self.nav2_file is not None:
                self.nav2_popup()
        elif keymap.matches('global', 'activate', key):
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
                    self.say(course.open_course(self))
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
        """Flip the Agents setting (block: the desk refuses new agent claims)."""
        self.settings.agents = 'block' if self.settings.agents == 'allow' else 'allow'
        self.say(f'Agents {self.settings.agents}')
        desk = self.desk
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
        if keymap.matches('global', 'activate', key):
            self.open('conditions')
