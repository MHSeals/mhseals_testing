#!/usr/bin/env python3
"""C07 installed-only gate. Run in a disposable network-none pinned ROS container.

Launch closure: actual navigation factory's planner_server action and static TF.
No controller, command publisher, hardware output node, or full launch executes.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
from types import SimpleNamespace

import yaml
from ament_index_python.packages import get_package_share_directory


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write(path, value):
    Path(path).write_text(json.dumps(value, indent=2, default=str) + '\n')


def factory(reference, fixture):
    from launch import LaunchContext
    share = Path(get_package_share_directory('mhseals_nav'))
    spec = importlib.util.spec_from_file_location('actual_navigation', share / 'launch/navigation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(dict(reference.arguments(), params_file=str(fixture),
        limits_file='', sim='false', use_lidar='false'))
    actions = module.navigation_nodes(context)
    from launch_ros.actions import Node
    nodes = [a for a in actions if isinstance(a, Node)]
    from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
    identity = lambda a: perform_substitutions(context, normalize_to_list_of_substitutions(a.node_executable))
    selected = [a for a in nodes if identity(a) == 'planner_server']
    assert len(selected) == 1
    from launch_ros.utilities import evaluate_parameters
    source = evaluate_parameters(context, selected[0]._Node__parameters)[0]
    target = Path('/scratch') / ('generated-%d.yaml' % len(list(Path('/scratch').glob('generated-*.yaml'))))
    shutil.copyfile(source, target)
    return selected[0], [identity(a) for a in nodes]


def run_launch(record, fixture):
    from launch import LaunchDescription, LaunchService
    from launch_ros.actions import Node
    from mhseals_testing.configuration import Selection
    planner, identities = factory(Selection.from_record(record), fixture)
    write('/scratch/action-identities.json', identities)
    service = LaunchService()
    service.include_launch_description(LaunchDescription([planner, Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x','0','--y','0','--z','0','--yaw','0','--pitch','0',
                   '--roll','0','--frame-id','map','--child-frame-id','base_link'])]))
    return service.run()


def spin_until(node, predicate, seconds=25):
    import rclpy
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError('bounded ROS wait exceeded')


def observe(record, fixture, label, cells=(32,24)):
    import rclpy
    from rclpy.node import Node
    from lifecycle_msgs.srv import ChangeState
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    log = open('/scratch/' + label + '-planner.log', 'w')
    process = subprocess.Popen([sys.executable, __file__, '--launch',
        json.dumps(record), str(fixture)], stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True)
    node = Node('c07_observer')
    try:
        client = node.create_client(ChangeState, '/planner_server/change_state')
        assert client.wait_for_service(timeout_sec=20), 'planner lifecycle unavailable'
        for transition in (1, 3):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            spin_until(node, future.done)
            assert future.result().success, ('lifecycle refused', transition)
        messages = []
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        node.create_subscription(OccupancyGrid, '/global_costmap/costmap', messages.append, qos)
        spin_until(node, lambda: bool(messages))
        message = messages[-1]
        result = dict(width=message.info.width, height=message.info.height,
            resolution=message.info.resolution, frame=message.header.frame_id,
            length=len(message.data), stamp=[message.header.stamp.sec,message.header.stamp.nanosec],
            data_sha256=hashlib.sha256(bytes((v % 256 for v in message.data))).hexdigest(),
            nodes=node.get_node_names_and_namespaces(), topics=node.get_topic_names_and_types())
        assert (result['width'], result['height'], result['resolution'], result['length'],
                result['frame']) == (*cells,0.25,cells[0]*cells[1],'map'), result
        assert not any('cmd_vel' in name or 'thrusters' in name for name, _ in result['topics'])
        write('/scratch/' + label + '-observation.json', result)
        return result
    finally:
        node.destroy_node()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
        log.close()


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from mhseals_testing.startup_owner import prepare_session
    from mhseals_hardware.configuration import load_defaults
    from mhseals_hardware.startup_config import node_startup_inputs
    from mhseals_hardware.thruster_mixer import validate_mixer, mix_thrusters
    from mhseals_hardware.velocity_law import Plant, VelocityLaw
    import mhseals_config, mhseals_hardware, mhseals_nav, mhseals_testing
    scratch = Path('/scratch')
    assert os.environ['ROS_DOMAIN_ID'] == '227'
    assert os.environ['ROS_LOCALHOST_ONLY'] == '1'
    assert not (scratch / 'store').exists(), 'requires empty root'
    os.environ['MHSEALS_CONFIG_ROOT'] = str(scratch / 'store')
    origins = {m.__name__: m.__file__ for m in
               (mhseals_config,mhseals_hardware,mhseals_nav,mhseals_testing)}
    assert all('/private-install/' in p for p in origins.values()), origins
    fixture = scratch / 'navigation.yaml'
    hardware = scratch / 'hardware.yaml'
    hw = load_defaults()
    hw['command_timeout'] = 0.73
    hw['thruster_matrix'][0] = -0.5
    axes = ('surge','sway','yaw')
    hw['characterization'] = dict(velocity_gain={a:{b:2.0 if a==b else 0.0 for b in axes} for a in axes},
        motion_limits={'velocity': dict.fromkeys(axes, 1.0)})
    hardware.write_text(yaml.safe_dump(hw))
    share = Path(get_package_share_directory('mhseals_nav'))
    nav = yaml.safe_load((share / 'config/nav2_params.yaml').read_text())
    grid = nav['global_costmap']['global_costmap']['ros__parameters']
    grid.update(width=8,height=6,resolution=0.25,use_sim_time=False,
                plugins=['inflation_layer'],always_send_full_costmap=True)
    fixture.write_text(yaml.safe_dump(nav))
    captured_hashes = dict(hardware=sha(hardware),navigation=sha(fixture))
    profile = SimpleNamespace(outputs='topic',components={'nav':dict(
        launch=['mhseals_nav','navigation.launch.py'], args=dict(params_file=str(fixture),
        sim='false',use_lidar='false'))})
    directory = scratch / 'session'
    directory.mkdir()
    write(directory / 'session.json', {'session':'c07'})
    args = SimpleNamespace(config=str(hardware))
    reference = prepare_session(profile,args,directory,'own',{})
    selected = reference.validate()
    assert len(selected.documents) == 4
    assert len(list((scratch/'store').glob('*/active.json'))) == 1
    doc = selected.documents['hardware-pure']
    assert doc['baseline']['output_backend'] == 'odroid'
    assert doc['effective']['output_backend'] == 'topic'
    rclpy.init()
    def hardware_seam(ref):
        host = Node('c07_parameter_host',parameter_overrides=[Parameter(k,value=v)
                    for k,v in ref.arguments().items()])
        try:
            effective, selection = node_startup_inputs(host,str(hardware))
            assert selection.ticket.candidate == reference.candidate
            assert effective['command_timeout'] == 0.73
            mixed = mix_thrusters(0.4,0,0,validate_mixer(effective['thruster_matrix']))
            assert tuple(mixed) == (-0.2,-0.4,0.4,0.4), mixed
            law = VelocityLaw(Plant(effective))
            law.observe((0,0,0),0,10)
            effort = law.step((0.2,0,0),10)
            assert effort == (0.26,0.0,0.0), effort
            return dict(mixer=list(mixed),law=list(effort))
        finally:
            host.destroy_node()
    try:
        first = hardware_seam(reference)
        planner, identities = factory(reference,fixture)
        del planner
        before = observe(reference.record(),fixture,'initial')
        hardware.write_text('invalid: [')
        fixture.unlink()
        adopted = prepare_session(profile,args,directory,'adopt',{})
        assert adopted == reference
        assert hardware_seam(adopted) == first
        after = observe(adopted.record(),fixture,'restart')
        assert before['data_sha256'] == after['data_sha256']
        negatives = []
        from dataclasses import replace
        bad = replace(reference,candidate='0'*64)
        for seam in (hardware_seam,lambda r: factory(r,fixture)):
            try:
                seam(bad)
            except (ValueError,FileNotFoundError) as error:
                negatives.append(str(error))
            else:
                raise AssertionError('bad candidate accepted')
        # A copied corrupt artifact must refuse in both startup seams before launch.
        shutil.copytree(scratch/'store',scratch/'corrupt-store')
        corrupt = replace(reference,root=str(scratch/'corrupt-store'))
        artifact = scratch/'corrupt-store'/reference.scope/'initializing/hardware-pure.json'
        artifact.write_text('{}')
        original_files = sorted(str(p) for p in (scratch/'corrupt-store').rglob('*'))
        for seam in (hardware_seam,lambda r: factory(r,fixture)):
            try:
                seam(corrupt)
            except (ValueError,FileNotFoundError) as error:
                negatives.append(str(error))
            else:
                raise AssertionError('corrupt artifact accepted')
        assert original_files == sorted(str(p) for p in (scratch/'corrupt-store').rglob('*'))
        # Fresh owner captures authoring changes; same-session restart above did not.
        hardware.write_text(yaml.safe_dump(hw))
        grid.update(width=10,height=4)
        fixture.write_text(yaml.safe_dump(nav))
        fresh_dir = scratch/'fresh-session'
        fresh_dir.mkdir()
        write(fresh_dir/'session.json', {'session':'c07-fresh'})
        fresh = prepare_session(profile,args,fresh_dir,'own',{})
        assert fresh.scope != reference.scope and fresh.candidate != reference.candidate
        fresh_grid = observe(fresh.record(),fixture,'fresh',(40,16))
        files = [p for p in (scratch/'store').rglob('*') if p.is_file()]
        write(scratch/'result.json',dict(selection=reference.record(),origins=origins,
            participants=sorted(selected.documents),baseline=str(selected.manifest),
            captured_hashes=captured_hashes,pure_hardware=first,
            initial=before,restart=after,negative_cases=negatives,
            fresh_selection=fresh.record(),fresh=fresh_grid,
            generated_hashes={str(p):sha(p) for p in scratch.glob('generated-*.yaml')},
            action_identities=identities,store_hashes={str(p):sha(p) for p in files}))
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv)>1 and sys.argv[1]=='--launch':
        sys.exit(run_launch(json.loads(sys.argv[2]),sys.argv[3]))
    main()
