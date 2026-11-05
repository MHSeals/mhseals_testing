"""The desk's settings: boat_agent set KEY=VALUE (any tunable, as the operator's Settings change, under the agent's
claim and between runs), live Nav2 keys through nav2/live.py, and the settings a run carries."""

import math
import time
from pathlib import Path

from mhseals_testing.agent import ask, layers
from mhseals_testing.agent import config as agent_config
from mhseals_testing.harness.settings import BY_KEY
from mhseals_testing.ui import agent_acts
from mhseals_testing.ui.feeds.agent_desk.common import NAV2_PREFIX

AGENTS_BLOCKED = ('operator must set Agents to allow: the TUI setting row agents (Control group) is block (reset to allow '
                  'at each start); the operator selects it and steps it with left/right')


def unknown_hint(key):
    return (f'unknown setting {key!r}: a session setting (harness/settings.py), a hardware key (mhseals_hardware '
            'config/default.yaml: manual_amplitude, decoupling, decoupling.yaw, command_mux.ramp, ...) or '
            'nav2.NODE.PARAM')


def _close(a, b):
    """Equal, or numbers (and lists of them) equal within float noise."""
    if isinstance(a, (list, tuple)) and isinstance(b, (list, tuple)):
        return len(a) == len(b) and all(_close(x, y) for x, y in zip(a, b))
    try:
        return math.isclose(float(a), float(b), abs_tol=1e-9)
    except (TypeError, ValueError):
        return a == b


class SettingsMixin:
    """set KEY=VALUE, nav2.NODE.PARAM=V and a run's --set values."""

    def do_set(self, req):
        """boat_agent set KEY=VALUE: any tunable setting (harness/settings.py, owner decision 2026-11-01), as the
        operator's Settings change, under the agent's claim and between runs; armed-unsafe keys (reload.yaml, and
        never amplitude or command_mode) only while disarmed. ``nav2.NODE.PARAM=V`` sets a live Nav2 key."""
        app = self.app
        if app.settings.agents == 'block':
            return self.reply(req, False, 'blocked', hint=AGENTS_BLOCKED)
        asked = req['args'].get('settings') or {}
        if not isinstance(asked, dict) or not asked:
            return self.reply(req, False, 'bad_args', hint='set KEY=VALUE [KEY=VALUE ...] (or set KEY to read it)')
        if any(v is None for v in asked.values()):
            if not all(v is None for v in asked.values()):
                return self.reply(req, False, 'bad_args', hint='read (set KEY) and write (set KEY=VALUE) separately')
            return self.read_settings(req, list(asked))
        if not req.get('claim'):
            return self.reply(req, False, 'unknown_claim', hint='set needs your claim: --claim ID')
        found = self.claim_state(req['claim'], req['who'])
        if found:
            return self.reply(req, False, found, hint='claim again' if found != 'not_holder' else 'use your own claim')
        nav2 = {str(k)[len(NAV2_PREFIX):]: v for k, v in asked.items() if str(k).startswith(NAV2_PREFIX)}
        values, hardware = {}, {}
        for key, value in asked.items():
            key = str(key)
            if key.startswith(NAV2_PREFIX):
                continue
            if key not in BY_KEY and layers.split(key) is None:
                return self.reply(req, False, 'bad_args', hint=unknown_hint(key))
            if key not in BY_KEY:
                hardware[key], refusal = self.hardware_refusal(key, value)
            else:
                values[key], refusal = agent_config.tunable_refusal(key, value, app.armed,
                                                                    current=getattr(app.settings, key, None))
            if refusal:
                return self.reply(req, False, refusal[0], hint=refusal[1],
                                  result={'ask': refusal[2]} if len(refusal) > 2 else None)
        if app.running or app.manual or self.act is not None or self.busy:
            return self.reply(req, False, 'busy', hint='a procedure, manual drive or act is running: set between '
                                                       'runs, never mid-segment; again once status .run is idle')
        result = {}
        if nav2:
            ok, error, hint, applied = self.set_nav2(req, nav2)
            if not ok:
                return self.reply(req, False, error, hint=hint, result=applied)
            result['nav2'] = applied
        if hardware:
            ok, hint, rows = self.set_hardware(req, hardware)
            result['hardware'] = rows
            if not ok:
                return self.reply(req, False, 'refused', hint=hint, result=result)
        if values:
            old = {key: getattr(app.settings, key, None) for key in values}
            error = self.apply_settings(values, req['who'])
            if error:                     # command_mode's hot switch refused (switch_command_mode): nothing set
                return self.reply(req, False, 'refused', hint=error, result=result)
            result['set'] = values
            result['rows'] = [dict(self.settings_row(key), old=old[key], requested=values[key],
                                   revert=ask.set_command(key, old[key], claim=req['claim'], why=f'revert {key}'))
                              for key in values]
        self.reply(req, True, result=result)

    def settings_row(self, key):
        """A session setting as it is now: value (the TUI's own, read back), where it lives, its range."""
        spec = BY_KEY[key]
        row = {'key': key, 'layer': 'settings', 'value': getattr(self.app.settings, key, None),
               'where': agent_config.settings_where(key), 'allowed': spec.describe(), 'tunable': spec.tunable,
               'armed_unsafe': agent_config.armed_unsafe(key)}
        if key == 'command_mode':        # the command path the session drives now, not just the setting
            row['command_units'] = getattr(self.app.profile, 'command_units', None)
        return row

    def hardware_path(self):
        return getattr(getattr(self.app, 'args', None), 'config', None)

    def hardware_row(self, key):
        top, _ = layers.split(key)
        return {'key': key, 'layer': 'hardware', 'value': layers.current(key, self.hardware_path()),
                'where': f'the operator hardware YAML {layers.config_file(self.hardware_path())}',
                'armed_unsafe': agent_config.armed_unsafe(top, surface='hardware')}

    def read_settings(self, req, keys):
        """``set KEY``: each key's value now, its layer and where it lives; no claim, nothing changes."""
        rows = []
        for key in map(str, keys):
            try:
                if key in BY_KEY:
                    rows.append(self.settings_row(key))
                elif layers.split(key) is not None:
                    rows.append(self.hardware_row(key))
                else:
                    return self.reply(req, False, 'bad_args', hint=unknown_hint(key))
            except (KeyError, ValueError, OSError) as error:
                return self.reply(req, False, 'bad_args', hint=f'{key}: {error}')
        self.reply(req, True, result={'read': rows})

    def hardware_refusal(self, key, value):
        """``(plan, None)`` for a hardware key (agent/layers.py), else ``(None, (error, hint[, ask]))``."""
        try:
            change = layers.plan(key, value, self.hardware_path())
        except (ValueError, OSError) as error:
            return None, ('bad_args', str(error))
        if self.app.armed and agent_config.armed_unsafe(change['top'], surface='hardware'):
            return None, agent_config.armed_refusal(key, change['old'], change['new'],
                                                    f"the operator hardware YAML {change['file']}")
        return change, None

    def set_hardware(self, req, planned):
        """Write each key through the TUI's own hardware save (history, reload, the driver's confirmation), after a
        snapshot of the file; read back from the file and, when it runs, from the driver. ``(ok, hint, rows)``."""
        app, who, rows = self.app, req['who'], []
        path = self.hardware_path()
        directory = getattr(getattr(app, 'paths', None), 'directory', None) or layers.config_file(path).parent
        remote = getattr(getattr(app, 'arming', None), 'remote', None)
        if remote is not None:          # a station TUI arming a remote boat: the boat's own TUI writes its YAML
            key, change = next(iter(planned.items()))
            host = remote.host.name
            return False, ask.text(ask.setting_ask(
                key, change['old'], change['new'], f"the boat's hardware YAML on {host}",
                "this TUI arms a remote boat, whose hardware.yaml is the config (read again at each arm)",
                command=ask.set_command(key, change['new'], claim='ID', host=host))), rows
        for key, change in planned.items():
            snapshot = layers.snapshot(path, Path(directory) / 'settings_snapshots', who, time.time_ns())
            if not app.save_hardware(**change['change']):
                return False, f'{key} not saved: {app.notice or "the hardware save refused it"}', rows
            row = dict(self.hardware_row(key), old=change['old'], requested=change['new'], snapshot=snapshot,
                       revert=ask.set_command(key, change['old'], claim=req.get('claim') or 'ID',
                                              why=f'revert {key}'))
            watched = getattr(getattr(app, 'watcher', None), 'watches', {}).get('hardware')
            row['applied'] = (getattr(watched, 'warning', None) or getattr(watched, 'error', None)
                              or ('applied' if app.armed else 'saved: the driver starts from it'))
            row['driver'] = self.driver_value(change['top'])
            rows.append(row)
            if row['requested'] is not None and not _close(row['value'], row['requested']):
                return False, f"{key} reads back {ask.show(row['value'])}, not {ask.show(row['requested'])}", rows
            app.emit('setting', key=key, old=change['old'], value=change['new'], by='agent', who=who,
                     layer='hardware')
            app.say(f"agent {who} set {key}={ask.show(change['new'])}")
            toasts = getattr(app, 'toasts', None)
            if toasts is not None:
                toasts.push(f'agent.setting.{key}', 'warn', app.notice, app.monitor.clock())
        return True, '', rows

    def driver_value(self, top):
        """The running driver's own value of ``top`` (its parameter), or None: not running, no ROS link."""
        if not self.app.armed or getattr(getattr(self.app, 'ros', None), 'node', None) is None:
            return None
        try:
            from mhseals_testing.ros.arming import DRIVER
            from mhseals_testing.ros.params import Params
            return Params(self.app.ros).get(f'/{DRIVER}', [top], timeout=2.0).get(top)
        except Exception:        # noqa: BLE001 - a readback that cannot answer is reported as None, never raised
            return None

    def set_nav2(self, req, requested):
        """Live Nav2 keys (``nav2.NODE.PARAM``) through nav2/live.py, the TUI's own apply path: preview,
        fences, a tokened hold, set, read back, roll back on any doubt. Off the key loop. ``(ok, error, hint,
        result)``."""
        import yaml
        live = self.app.live_nav2() if hasattr(self.app, 'live_nav2') else None
        if live is None:
            return False, 'refused', 'no live Nav2 here (a watcher, a review or no Nav2 profile)', None
        values = {k: yaml.safe_load(v) if isinstance(v, str) else v for k, v in requested.items()}   # typed request values, not a file
        live.claim = req.get('claim')
        title = f"Agent {req['who']}: setting Nav2 {', '.join(values)}"
        def work():
            receipt = live.apply(values)
            return receipt, (live.readback(values) if hasattr(live, 'readback') else {})
        try:
            receipt, now = agent_acts.in_background(self.app, title, work, message=title)
        except Exception as fault:      # noqa: BLE001 - a Nav2 timeout: told, never raised
            return False, 'refused', f'{type(fault).__name__}: {fault}', None
        status = receipt.get('status')
        changed = {r['key']: r for r in receipt.get('rows', ())}
        # one row per requested key, read back from the node after the apply: an unchanged key says so
        rows = [{'key': key, 'requested': value, 'readback': now.get(key),
                 **({'old': changed[key]['old'], 'new': changed[key]['new']} if key in changed else {'unchanged': True})}
                for key, value in values.items()]
        rows += [{'key': k, 'old': r['old'], 'new': r['new'], 'derived': True} for k, r in changed.items() if k not in values]
        self.app.emit('agent_nav2', who=req['who'], keys=sorted(values), status=status,
                      readback={r['key']: r['readback'] for r in rows if 'readback' in r})
        if status != 'verified':
            why = receipt.get('error') or '; '.join(receipt.get('reasons') or []) or str(receipt.get('refused'))
            return False, 'refused', f'nav2 {status}: {why}', {'rows': rows}
        return True, '', '', {'rows': rows, 'reset': receipt.get('reset')}

    def settings_of(self, req, allowed, idle=True):
        """(``{key: parsed value}``, None) for the request's ``settings``, or (None, (error, hint)): an unknown,
        unlisted, never or out-of-choices key, or with ``idle`` any while a procedure, act or manual drive runs."""
        asked = req['args'].get('settings') or {}
        if not isinstance(asked, dict):
            return None, ('bad_args', 'settings is {key: value}')
        values = {}
        for key, value in asked.items():
            values[key], refusal = agent_config.setting_refusal(str(key), value, allowed)
            if refusal:
                return None, refusal
        app = self.app
        if idle and values and (app.running or app.manual or self.act is not None or self.busy):
            return None, ('busy', 'a procedure or act is running: settings change between runs, never mid-segment')
        return values, None

    def apply_settings(self, values, who):
        """Each value as the operator's change; the first refusal (command_mode's hot switch) stops there."""
        for key, value in values.items():
            error = self.app.set_setting(key, value, who=who)
            if error:
                return error
        return ''
