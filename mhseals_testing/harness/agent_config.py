"""config/agent.yaml: agent leases, request deadlines, the procedures agents may run, the hook. ROS-free.

``load(path)`` parses and validates the file strictly and returns a plain dict with every key
filled. ``Watch`` is the hot reload: ``get()`` re-reads when the file changes and keeps the last
good config (``error`` names what is wrong with the new one).
"""

import os
import re
from pathlib import Path

from mhseals_testing.strict_yaml import load_strict

import yaml

SCHEMA = 'mhseals.agent_config/1'
LEASE_KEYS = ('idle_s', 'motion_s', 'max_claim_s')
NUMBERS = ('request_s', 'confirm_s', 'ask_s')
DRIVERS = ('warn', 'hold')
HOOK_LISTS = ('match', 'skip', 'deny')


def config_path():
    """config/agent.yaml in the source tree, else the installed share directory."""
    source = Path(__file__).resolve().parents[2] / 'config' / 'agent.yaml'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'agent.yaml'


def load(path=None):
    """The validated config; ValueError (naming the key) on anything wrong."""
    path = Path(path or config_path())
    return validate(load_strict(path.read_text(), path))


def validate(data):
    if not isinstance(data, dict) or data.get('schema') != SCHEMA:
        raise ValueError(f'agent.yaml: schema must be {SCHEMA}')
    known = {'schema', 'lease', 'run', 'unclaimed_driver', 'hook', *NUMBERS}
    if (extra := sorted(set(data) - known)):
        raise ValueError(f'agent.yaml: unknown key {extra[0]}')
    lease = data.get('lease')
    if not isinstance(lease, dict) or set(lease) != set(LEASE_KEYS):
        raise ValueError(f"agent.yaml: lease needs exactly {', '.join(LEASE_KEYS)}")
    for key, value in [*(('lease.' + k, lease[k]) for k in LEASE_KEYS), *((k, data.get(k)) for k in NUMBERS)]:
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value <= 0:
            raise ValueError(f'agent.yaml: {key} must be a positive number of seconds')
    if not lease['motion_s'] <= lease['idle_s'] <= lease['max_claim_s']:
        raise ValueError('agent.yaml: lease needs motion_s <= idle_s <= max_claim_s')
    run = data.get('run')
    if not isinstance(run, list) or not all(isinstance(k, str) and k for k in run):
        raise ValueError('agent.yaml: run must be a list of procedure keys')
    if data.get('unclaimed_driver') not in DRIVERS:
        raise ValueError(f"agent.yaml: unclaimed_driver must be one of {'/'.join(DRIVERS)}")
    hook = data.get('hook')
    if not isinstance(hook, dict) or not isinstance(hook.get('target'), str) \
            or not isinstance(hook.get('path'), str):
        raise ValueError('agent.yaml: hook needs target and path')
    for key in HOOK_LISTS:
        patterns = hook.get(key, [])
        if not isinstance(patterns, list) or not all(isinstance(p, str) for p in patterns):
            raise ValueError(f'agent.yaml: hook.{key} must be a list of regexes')
        for pattern in patterns:
            try:
                re.compile(pattern)
            except re.error as error:
                raise ValueError(f'agent.yaml: hook.{key} {pattern!r}: {error}') from None
    limit = hook.get('max_per_10s', 6)
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
        raise ValueError('agent.yaml: hook.max_per_10s must be a positive integer')
    return {**data, 'lease': dict(lease), 'run': list(run),
            'hook': {**hook, **{k: list(hook.get(k, [])) for k in HOOK_LISTS}, 'max_per_10s': limit}}


class Watch:
    """The current config, re-read when the file's (mtime, size, inode) changes."""

    def __init__(self, path=None):
        self.path = Path(path or config_path())
        self.key, self.config, self.error = None, None, None
        self.get()
        if self.config is None:
            raise ValueError(self.error)   # no last good config to fall back on at start

    def get(self):
        try:
            st = os.stat(self.path)
            key = (st.st_mtime_ns, st.st_size, st.st_ino)
        except OSError as error:
            self.error = f'agent.yaml: {error.strerror}'
            return self.config
        if key != self.key:
            self.key = key
            try:
                self.config, self.error = load(self.path), None
            except (OSError, ValueError, yaml.YAMLError) as error:   # keep the last good one
                self.error = str(error).splitlines()[0] if str(error) else type(error).__name__
        return self.config
