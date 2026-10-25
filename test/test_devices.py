"""The device registry and the part -> host map derived from the device profiles."""

import pytest

from mhseals_testing import devices
from mhseals_testing.devices import DeviceError

ENV = '''# Boat LAN
BOAT_SUBNET=192.168.0.0/24
ROUTER_IP=192.168.0.1
GATEWAY_IP=192.168.0.2
LIDAR_IP=192.168.0.201
STATIC_LEASES="56:e2:03:5c:60:22=blastoise-odroid=192.168.0.152 3c:6d:66:bf:6b:e2=blastoise-jetson=192.168.0.153"
'''
DEPLOYMENT = '''
roles:
  operator: {profile: operator}
hosts:
  blastoise-odroid: {roles: [boat_computer], capabilities: [thrusters, mavros]}
  blastoise-jetson: {roles: [camera_host], capabilities: [gpu, zed, lidar], via: tranquility}
  tranquility: {roles: [operator, gateway], user: peace, address: $GATEWAY_IP}
  router: {roles: [router], address: $ROUTER_IP, ssh: false}
  vlp16: {roles: [lidar_sensor], address: $LIDAR_IP, ssh: false}
ping: [camera_host, router, lidar_sensor]
'''
PROFILES = {
    'odroid': {'requires': ['mavros'], 'parts': {
        'odometry': {'launch': 'odom.launch.py', 'nodes': ['robot_state_publisher']},
        'ekf': {'launch': 'ekf.launch.py'},
        'navigation': {'launch': 'navigation.launch.py', 'kind': 'commander'}}},
    'jetson': {'requires': ['zed', 'gpu'], 'parts': {
        'camera': {'launch': 'camera.launch.py', 'nodes': ['zed_node']},
        'detections': {'launch': 'objects.launch.py'}}},
    'operator': {'parts': {'rtk_base': {'run': ['mhseals_testing', 'rtk_base']}}},
    'sim': {'parts': {'odometry': {'launch': 'odom.launch.py'}, 'ekf': {'launch': 'ekf.launch.py'},
                      'camera': {'launch': 'camera.launch.py', 'enabled': False},
                      'navigation': {'launch': 'navigation.launch.py', 'kind': 'commander'}}},
}


@pytest.fixture
def files(tmp_path):
    (tmp_path / 'boat-network.env').write_text(ENV)
    (tmp_path / 'deployment.yaml').write_text(DEPLOYMENT)
    return tmp_path


def load(files, operator='', deployment=True):
    (files / 'operator.yaml').write_text(operator)
    return devices.load(deployment=files / ('deployment.yaml' if deployment else 'none.yaml'),
                        operator=files / 'operator.yaml', env=files / 'boat-network.env')


def test_package_layer_alone_puts_every_role_on_this_machine(files):
    registry = load(files, deployment=False)
    me = devices.whoami(registry, set())
    bare = {name: {'parts': profile['parts']} for name, profile in PROFILES.items()}
    parts = devices.assign(registry, me, bare.get, wanted=['odometry', 'camera', 'rtk_base'])
    assert {part.host.name for part in parts.values()} == {'local'}
    assert {devices.reach(part.host, me) for part in parts.values()} == {'local'}
    # The package host claims no capabilities: boat profiles that need hardware are refused there.
    with pytest.raises(DeviceError, match=r"odroid requires \['mavros'\]; host local has none"):
        devices.assign(registry, me, PROFILES.get)


def test_addresses_come_from_leases_env_keys_and_mdns(files):
    hosts = load(files).hosts
    assert (hosts['blastoise-jetson'].address, hosts['blastoise-jetson'].source) == ('192.168.0.153', 'lease')
    assert (hosts['tranquility'].address, hosts['tranquility'].source) == ('192.168.0.2', 'env')
    assert hosts['vlp16'].address == '192.168.0.201'
    assert hosts['blastoise-jetson'].user == 'roboboat'                  # package defaults
    assert hosts['blastoise-jetson'].container == 'roboboat_dev'


def test_a_host_without_a_lease_falls_back_to_mdns(files):
    registry = load(files, operator='hosts: {spare-orin: {roles: [spare]}}')
    assert (registry.hosts['spare-orin'].address, registry.hosts['spare-orin'].source) == ('spare-orin.local', 'mdns')


def test_a_missing_env_key_is_a_load_error(files):
    with pytest.raises(DeviceError, match=r'\$RADIO_IP is not set'):
        load(files, operator='hosts: {radio: {roles: [radio], address: $RADIO_IP}}')


def test_derived_map_joins_profile_role_host_and_reach(files):
    registry = load(files)
    me = devices.whoami(registry, {'192.168.0.152'})
    assert me.name == 'blastoise-odroid'
    parts = devices.assign(registry, me, PROFILES.get)
    assert parts['camera'].host.name == 'blastoise-jetson' and parts['camera'].role == 'camera_host'
    assert parts['navigation'].kind == 'commander' and parts['ekf'].kind == 'sensing'
    assert devices.reach(parts['odometry'].host, me) == 'local'
    assert devices.reach(parts['camera'].host, me) == 'ssh -J tranquility roboboat@192.168.0.153 docker roboboat_dev'
    assert devices.reach(parts['rtk_base'].host, me) == 'ssh peace@192.168.0.2 docker roboboat_dev'
    assert [row[0] for row in devices.table(parts, me)] == [
        'odometry', 'ekf', 'navigation', 'camera', 'detections', 'rtk_base']


def test_the_fallback_host_only_takes_roles_nobody_else_claims(files):
    registry = load(files)
    assert registry.host_for('camera_host').name == 'blastoise-jetson'
    registry = load(files, operator='hosts: {tranquility: {roles: [gateway]}}')
    assert registry.host_for('operator').name == 'local'


def test_a_role_on_two_hosts_is_refused(files):
    registry = load(files, operator='hosts: {spare: {roles: [camera_host], capabilities: [zed, gpu]}}')
    with pytest.raises(DeviceError, match=r"role 'camera_host' needs exactly one host \(has blastoise-jetson, spare\)"):
        devices.assign(registry, devices.whoami(registry), PROFILES.get)


def test_a_part_claimed_by_two_profiles_is_refused(files):
    profiles = dict(PROFILES, jetson={'requires': [], 'parts': {'ekf': {'launch': 'ekf.launch.py'}}})
    with pytest.raises(DeviceError, match=r"part 'ekf' is claimed by both odroid and jetson"):
        devices.assign(load(files), devices.Host('x'), profiles.get)


def test_a_wanted_part_nobody_claims_is_refused(files):
    with pytest.raises(DeviceError, match='no active device profile runs mission'):
        devices.assign(load(files), devices.Host('x'), PROFILES.get, wanted=['odometry', 'mission'])


def test_a_host_lacking_a_required_capability_is_refused(files):
    # Perception moved to the ODROID, which has no ZED.
    registry = load(files, operator='hosts: {blastoise-jetson: null, blastoise-odroid: '
                                    '{roles: [boat_computer, camera_host]}}')
    with pytest.raises(DeviceError, match=r"jetson requires \['gpu', 'zed'\]; host blastoise-odroid"):
        devices.assign(registry, devices.Host('x'), PROFILES.get)


def test_moving_a_role_moves_its_parts(files):
    registry = load(files, operator='hosts: {blastoise-jetson: null, blastoise-odroid: '
                                    '{roles: [boat_computer, camera_host], capabilities: [mavros, zed, gpu]}}')
    me = devices.whoami(registry, {'192.168.0.152'})
    parts = devices.assign(registry, me, PROFILES.get)
    assert devices.reach(parts['camera'].host, me) == 'local'


def test_placement_runs_the_named_profiles_here(files):
    registry = load(files)
    me = devices.whoami(registry, {'192.168.0.2'})
    parts = devices.assign(registry, me, PROFILES.get, placement={'self': ['sim']}, wanted=['odometry', 'camera'])
    assert {part.host.name for part in parts.values()} == {'tranquility'}
    assert not parts['camera'].enabled and parts['camera'].role is None
    assert devices.table(parts, me)[2][0] == 'camera (disabled)'


def test_operator_self_wins_over_addresses(files):
    registry = load(files, operator='self: tranquility')
    assert devices.whoami(registry, {'192.168.0.152'}).name == 'tranquility'


def test_ping_targets_skip_this_machine(files):
    registry = load(files)
    me = devices.whoami(registry, {'192.168.0.153'})
    assert devices.ping_targets(registry, me) == {'router': '192.168.0.1', 'vlp16': '192.168.0.201'}


@pytest.mark.parametrize('text, message', [
    ('hosts: {a: {rolez: [x]}}', 'keys from'),
    ('hosts: {a: {capabilities: [laser]}}', "unknown \\['laser'\\]"),
    ('hosts: {a: {ssh: maybe}}', 'true or false'),
    ('roles: {camera_host: jetson}', 'profile'),
    ('self: ghost', "unknown host 'ghost'"),
    ('hosts: {a: {via: ghost}}', "via: unknown host 'ghost'"),
    ('frobs: 1', 'keys from'),
    ('hosts: {a: {}, a: {}}', 'duplicate key'),
    # Everything that reaches ssh's argv or the remote shell is a plain name.
    ('hosts: {"a;reboot": {}}', 'not a plain name'),
    ('hosts: {a: {container: "x; rm -rf ~"}}', 'container: .* not a plain name'),
    ('hosts: {a: {container: "$(reboot)"}}', 'not a plain name'),
    ('hosts: {a: {user: "-oProxyCommand=sh"}}', 'user: .* not a plain name'),
    ('hosts: {a: {address: "-oProxyCommand=touch x"}}', 'address: .* not a plain name'),
    ('hosts: {a: {address: "10.0.0.1 x"}}', 'not a plain name'),
    ('hosts: {a: {via: "b`id`"}}', 'via: .* not a plain name'),
    ('defaults: {container: "a|b"}', 'defaults.container: .* not a plain name'),
])
def test_bad_layers_are_refused_with_their_place(files, text, message):
    with pytest.raises(DeviceError, match=message):
        load(files, operator=text)


def test_an_env_address_that_is_not_a_plain_name_is_refused(files):
    (files / 'boat-network.env').write_text(ENV + 'GATEWAY_IP="192.168.0.2;reboot"\n')
    with pytest.raises(DeviceError, match=r'tranquility.address \(env\).*not a plain name'):
        load(files)


def test_env_parser_tolerates_comments_and_bad_quotes(tmp_path):
    path = tmp_path / 'boat-network.env'
    path.write_text('A=1  # note\nB="x y"\nC="unclosed\n# D=4\nnot a line\n')
    assert devices.env_values(path) == {'A': '1', 'B': 'x y'}
    assert devices.env_values(tmp_path / 'missing.env') == {}


def test_profiles_unavailable_reads_as_a_device_error(monkeypatch):
    monkeypatch.setitem(__import__('sys').modules, 'mhseals_nav.device_profile', None)
    with pytest.raises(DeviceError, match='device profiles are not installed'):
        devices.load_device_profile('jetson')
