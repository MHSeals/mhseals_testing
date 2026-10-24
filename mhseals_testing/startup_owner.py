"""Capture or adopt configuration after Registry/session ownership is decided."""
import os
import uuid
from pathlib import Path

import yaml

from mhseals_hardware.configuration import config_path
from mhseals_testing import data, nav2_limits
from mhseals_testing.configuration import Selection, bootstrap, hardware, navigation
from mhseals_testing.harness import session
from mhseals_testing.ros.arming import controller_topics

NAV_LAUNCHES = {('mhseals_nav', 'navigation.launch.py'), ('mhseals_nav', 'robot.launch.py')}
RESERVED = {'config_root', 'config_scope', 'config_candidate'}


def navigation_args(profile, overlay):
    """Resolve the actual final navigation invocation; reject ambiguous closures."""
    invocations = []
    for key, spec in profile.components.items():
        if tuple(spec.get('launch', ())) not in NAV_LAUNCHES:
            continue
        args = {**spec.get('args', {}), **overlay.get(key, {})}
        if RESERVED & args.keys():
            raise ValueError('configuration reference is reserved for the session owner')
        # These are the only configuration inputs supported by the current compiler.
        # robot's other arguments control components outside this snapshot closure.
        allowed = {'sim', 'use_lidar', 'params_file', 'limits_file'}
        if tuple(spec['launch']) == ('mhseals_nav', 'navigation.launch.py') and args.keys() - allowed:
            raise ValueError('unsupported managed navigation arguments')
        if tuple(spec['launch']) == ('mhseals_nav', 'robot.launch.py'):
            if 'params_file' in args:
                raise ValueError('robot launch uses nav2_params_file')
            if 'nav2_params_file' in args:
                args['params_file'] = args['nav2_params_file']
            args.setdefault('limits_file', str(nav2_limits.path()))
        if spec.get('limits'):
            args['limits_file'] = str(nav2_limits.path())
        invocation = {k: args[k] for k in allowed if k in args}
        invocation.setdefault('sim', 'true')
        invocation.setdefault('use_lidar', 'true')
        invocations.append(invocation)
    if len(invocations) > 1:
        raise ValueError('one navigation invocation per managed session is required')
    return invocations[0] if invocations else None


def selected_domains(profile, overlay):
    return tuple(sorted((*(['hardware'] if profile.outputs != 'none' else []),
                         *(['navigation'] if navigation_args(profile, overlay) is not None else []))))


def prepare_session(profile, args, directory, mode, overlay):
    """No allocation or authoring reads on adoption/watch; legacy adoption refuses."""
    domains = selected_domains(profile, overlay)
    header = data.read_json(Path(directory) / 'session.json')
    record = header.get('startup_selection') if isinstance(header, dict) else None
    if mode != 'own':
        if record is None and not domains:
            return None
        reference = Selection.from_record(record)
        if reference.domains != domains:
            raise ValueError('selected domains differ; create a new session')
        selected = reference.validate()
        if 'hardware' in domains and (selected.documents['hardware-pure']['effective']['output_backend']
                                      != profile.outputs):
            raise ValueError('output backend differs; create a new session')
        if 'hardware' in domains:
            controller_topics(profile, selected.documents['hardware-pure']['effective'])
        if 'navigation' in domains:
            invocation = navigation_args(profile, overlay)
            captured = selected.documents[navigation.PARTICIPANT]['invocation']
            if any(navigation._boolean(invocation[k]) != captured[k] for k in ('sim', 'use_lidar')):
                raise ValueError('navigation invocation differs; create a new session')
        return reference
    if record is not None:
        raise ValueError('fresh owner cannot overwrite an existing session selection')
    if not domains:
        return None
    root = str(Path(navigation.snapshot_root()).expanduser().absolute())
    scope = uuid.uuid4().hex
    prepared = {}
    if 'hardware' in domains:
        path = config_path(args.config)
        explicit = args.config or os.environ.get('MHSEALS_HARDWARE_CONFIG')
        operator = path if explicit or path.exists() else None
        invocation = {'output_backend': profile.outputs}
        topics = controller_topics(profile)
        if topics:
            invocation['controller'] = topics
        prepared['hardware'] = hardware.prepare_inputs(operator, session=(
            yaml.safe_dump(invocation),
            f"testing profile {getattr(profile, 'name', 'invocation')} controller topology"))
    if 'navigation' in domains:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('mhseals_nav'))
        invocation = navigation_args(profile, overlay)
        prepared['navigation'] = navigation.prepare_inputs(
            root, scope, invocation.get('params_file', share / 'config/nav2_params.yaml'),
            invocation.get('limits_file'), sim=invocation['sim'],
            use_lidar=invocation['use_lidar'], resources=navigation.resource_paths(share))
    reference = bootstrap(root, scope, prepared)
    session.persist_startup_selection(directory, reference.record())
    return reference
