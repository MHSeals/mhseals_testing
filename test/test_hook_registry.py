"""The agent hook's boat hosts come from the device registry (D-20261104-03), and that blocks no less than the host
names it had in code: a behaviour lock against hook_lock.json (what origin/main 5dc2ce9e, the last hook with literal
hosts, answered for CORPUS), a renamed fleet, and an unread registry that fails closed."""

import json
from pathlib import Path

import pytest

from mhseals_testing import agent_hook as hook
from mhseals_testing.hook import hosts

import hook_registry
from test_agent_hook import ALLOW, RUNS

LOCK = Path(__file__).parent / 'fixtures' / 'hook_lock.json'
WHO = 'claude-3f2a'
LEASES = ('STATIC_LEASES="56:e2:03:5c:60:22=blastoise-odroid=192.168.0.152 '
          '3c:6d:66:bf:6b:e2=blastoise-jetson=192.168.0.153"\n')
NAMES = ('blastoise', 'blastoise-odroid', 'blastoise-jetson', 'squirtle', 'squirtle-jetson', 'roboboat@blastoise-odroid',
         'devbox')
REMOTE = ('sudo reboot', 'docker stop roboboat_dev', 'systemctl --user restart mhseals-odometry', 'pkill -f x', 'kill 1234',
          'tmux kill-server', 'tmux -L boat send-keys -t boat q', 'ros2 launch mhseals_nav odom.launch.py',
          'ros2 topic pub /cmd_vel x y', 'uptime', 'tee ~/astro_dock/src/mhseals_nav/launch/x.launch.py',
          'shutdown -h now', f'tmux kill-session -t agent-{WHO}', '~/astro_dock/src/mhseals_testing/tools/boat_agent status')
CORPUS = ([f'ssh {n} {c}' for n in NAMES for c in REMOTE] + [f'ssh -t {n} "{c}"' for n in NAMES for c in REMOTE]
          + [f'ssh {n} bash <<EOF\n{c}\nEOF' for n in NAMES[:5] for c in REMOTE[:3]])

BOAT_AGENT = hook.DEFAULTS['boat_agent']


def answer(command):
    """``[deny route or None, host]`` and whether a TUI note goes out, as hook_lock.json records them."""
    found = hook.denied(command, hook.DEFAULTS, who=WHO)
    return [hook.use_of(found[0]) if found else None, found[1] if found else None,
            hook.matches(hook.note_match(hook.DEFAULTS), command) and not hook.matches(hook.DEFAULTS['skip'], command)]


@pytest.fixture
def fleet(tmp_path, monkeypatch):
    """``fleet(profiles)``: that profile set is the registry; this machine is devbox, the network env the boat's."""
    network = tmp_path / 'network.env'
    network.write_text(LEASES)
    monkeypatch.setattr(hosts, 'network_file', lambda: network)
    monkeypatch.setattr(hosts, 'hostname', lambda: 'devbox')
    monkeypatch.delenv('BOAT_AGENT_TARGET', raising=False)

    def use(profiles, name='profiles'):
        root = hook_registry.profile_set(tmp_path / name, profiles)
        hook_registry.use(monkeypatch, root)
        return root
    return use


def test_the_shipped_registry_blocks_everything_the_literal_hosts_did(fleet):
    fleet(hook_registry.SHIPPED)
    lock = json.loads(LOCK.read_text())
    assert sorted(lock['answers']) == sorted(CORPUS)
    for command, (route, host, noted) in lock['answers'].items():
        now = answer(command)
        if route:                              # denied then: denied now, by the same route, on the same host
            assert now[:2] == [route, host], command
        if noted:
            assert now[2], command
    for command in RUNS:
        assert hook.denied(command, hook.DEFAULTS), command
    for command in ALLOW:
        assert hook.denied(command, hook.DEFAULTS) is None, command


def test_the_playbook_is_the_same_for_the_shipped_registry(fleet):
    fleet(hook_registry.SHIPPED)
    lock = json.loads(LOCK.read_text())
    assert hook.start_text(WHO, config=hook.DEFAULTS) == lock['playbook']
    assert hook.brief_text(WHO, config=hook.DEFAULTS) == lock['brief']
    reason = hook.deny_reason(*hook.denied('ssh blastoise pkill -f x', hook.DEFAULTS), who=WHO, config=hook.DEFAULTS)
    assert f'Use: ssh blastoise-odroid {BOAT_AGENT} stop' in reason       # the legacy alias, from the registry


def test_a_renamed_or_added_boat_host_is_blocked_without_a_code_edit(fleet):
    root = fleet(hook_registry.RENAMED)
    names = hook.boats(hook.DEFAULTS)
    assert names.hosts == ('heron-pc', 'kestrel-main', 'kestrel-cam') and names.read
    for host in ('heron-pc', 'heron', 'kestrel-main', 'kestrel-cam', 'kestrel', 'kestrel-spare', 'crew@heron-pc'):
        assert hook.denied(f'ssh {host} sudo reboot', hook.DEFAULTS)[0].host == host.split('@')[-1], host
        assert answer(f'ssh {host} uptime') == [None, None, '@' not in host], host       # a TUI note, as before
    for gone in ('squirtle-jetson', 'shore'):      # not boats of this fleet (the network env's leases still are)
        assert hook.denied(f'ssh {gone} sudo reboot', hook.DEFAULTS) is None, gone
    reason = hook.deny_reason(*hook.denied('ssh kestrel pkill -f x', hook.DEFAULTS), who=WHO, config=hook.DEFAULTS)
    assert f'Use: ssh kestrel-main {BOAT_AGENT} stop' in reason      # the boat computer, not the file's first
    assert 'BOAT_AGENT_TARGET=heron-pc in your environment (`heron` still works' in hook.start_text(WHO, config=hook.DEFAULTS)
    (root / 'kestrel' / 'devices.yaml').write_text(hook_registry.KESTREL + '  osprey2: {roles: [camera_host]}\n')
    assert hook.denied('ssh osprey2 sudo reboot', hook.DEFAULTS)[1] == 'osprey2'      # added: blocked at once
    computers_only = dict(hook.DEFAULTS, boat_roles=['boat_computer'])
    assert hook.denied('ssh osprey2 sudo reboot', computers_only) is None      # the roles are configuration


@pytest.mark.parametrize('broken', ['missing', 'unparsable', 'none'])
def test_an_unread_registry_fails_closed_and_names_the_route(fleet, monkeypatch, tmp_path, broken):
    if broken == 'missing':
        fleet(hook_registry.SHIPPED)
        monkeypatch.setenv('ASTRO_REGISTRY', str(tmp_path / 'gone.yaml'))      # named but missing
    elif broken == 'unparsable':
        fleet(dict(hook_registry.SHIPPED, blastoise=(hook_registry.VESSEL + 'registry: devices.yaml\n',
                                                     'hosts:\n  blastoise-odroid: {roles: [boat_computer,\n')))
    else:
        fleet({'fleet': ('# no vessel names a registry\n', None)})
    names = hook.boats(hook.DEFAULTS)
    assert not names.read
    found = hook.denied('ssh devbox uptime', hook.DEFAULTS, who=WHO)
    assert found and found[1] == 'devbox'
    reason = hook.deny_reason(*found, who=WHO, config=hook.DEFAULTS)
    assert 'cannot read the device registry' in reason and f'ssh devbox {BOAT_AGENT} <verb> --who {WHO}' in reason
    assert hook.use_of(found[0]) == 'boat_agent status'
    for command in ('ssh newboat sudo reboot', 'ssh -t devbox', 'ssh devbox bash <<EOF\nuptime\nEOF',
                    'ssh devbox boat_agent ask "$(uptime)" --who c1', 'ssh devbox "boat_agent status; uptime"'):
        assert hook.denied(command, hook.DEFAULTS, who=WHO), command
    for command in (f'ssh devbox {BOAT_AGENT} status --who {WHO}', 'ssh -t devbox boat_tui status',
                    f'ssh devbox tmux kill-session -t agent-{WHO}', 'ssh localhost uptime', 'ssh 127.0.0.1 ls',
                    'git status', 'ros2 topic echo /odom'):
        assert hook.denied(command, hook.DEFAULTS, who=WHO) is None, command
    assert hook.matches(hook.note_match(hook.DEFAULTS), 'ssh anyhost uptime')          # every ssh is boat-bound
    assert hook.this_boat(names) is None                                               # a laptop stays a laptop
    monkeypatch.setattr(hosts, 'DEVICE_TREE', tmp_path)                                  # a board computer:
    assert hook.this_boat(names) == 'devbox' and hook.denied('kill 4242', hook.DEFAULTS)  # deny_boat here too


def test_the_reader_matches_pyyaml_and_refuses_what_it_cannot_read():
    yaml = pytest.importorskip('yaml')
    package = Path(hook.__file__).resolve().parent.parent / 'config' / 'devices.yaml'
    for text in (hook_registry.BLASTOISE, hook_registry.SQUIRTLE, hook_registry.KESTREL, hook_registry.HERON,
                 package.read_text(), 'hosts: {local: {capabilities: [gpu]}}\n'):
        want = [(k, tuple((v or {}).get('roles', []))) for k, v in (yaml.safe_load(text).get('hosts') or {}).items()]
        assert hook.registry_hosts(text) == want
    for text in ('hosts:\n  a: {roles: [x,\n    y]}\n', 'hosts:\n  a: &x {roles: [x]}\n', 'hosts:\n\ta: {}\n',
                 'hosts: [a]\n', 'hosts:\n  a:\n    roles: x\n', 'hosts:\n  - a\n', 'hosts:\n  a: {}\n  a: {}\n',
                 'hosts:\n  a: {roles: [x]}\n b: {}\n', 'hosts:\n  a: {roles: "x\\"y"}\n'):
        with pytest.raises(ValueError):
            hook.registry_hosts(text)
