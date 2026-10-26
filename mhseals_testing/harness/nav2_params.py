"""Which Nav2 parameters a running stack applies by itself, and in what order. Pure, ROS-free.

Keys are flat ``<node>.<param>`` strings (``controller_server.FollowPath.vx_max``); the two
costmaps are ``local_costmap`` / ``global_costmap`` and may repeat the name as in the YAML
(``local_costmap.local_costmap.footprint``). The live keys are listed in config/nav2_classes.yaml;
every other key needs a Navigation reset. Applying a plan is harness/nav2_live.py.
"""

from fnmatch import fnmatchcase
from pathlib import Path

import yaml

# Apply order; the smoother is also sent first, at the more restrictive limits.
ORDER = ('velocity_smoother', 'controller_server', 'local_costmap', 'global_costmap',
         'planner_server', 'waypoint_follower', 'behavior_server', 'bt_navigator')
COSTMAPS = ('local_costmap', 'global_costmap')
FOLLOW = 'controller_server.FollowPath.'


def _rules_path():
    source = Path(__file__).resolve().parents[2] / 'config' / 'nav2_classes.yaml'
    if source.is_file():
        return source
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'nav2_classes.yaml'
    except (ImportError, KeyError):
        return source


def _ceilings():
    """mhseals_nav's ceilings (source tree, else installed share), strictly loaded: a missing or bad file raises."""
    from mhseals_nav import navigation_config, startup_projection
    path = startup_projection.ceilings_path()
    if not path.is_file():
        try:
            from ament_index_python.packages import get_package_share_directory
            path = startup_projection.ceilings_path(get_package_share_directory('mhseals_nav'))
        except (ImportError, KeyError):
            pass
    return navigation_config.load_ceilings(path)


def live_patterns(path=None):
    """Return the fnmatch patterns of the keys a running stack applies by itself."""
    return yaml.safe_load(Path(path or _rules_path()).read_text())['live']


def rank(node):
    """Position of ``node`` in the apply order."""
    return ORDER.index(node) if node in ORDER else len(ORDER)


def split(key):
    """('controller_server', 'FollowPath.vx_max') from a flat key."""
    node, _, param = key.partition('.')
    if node in COSTMAPS and param.startswith(node + '.'):
        param = param[len(node) + 1:]
    return node, param


def target(key):
    """(node name as ROS sees it, parameter name): costmaps are /<name>/<name>."""
    node, param = split(key)
    return (f'/{node}/{node}' if node in COSTMAPS else f'/{node}'), param


def is_live(key, patterns=None):
    """Whether a running stack applies ``key`` by itself (no reset)."""
    flat = '.'.join(split(key))
    return any(fnmatchcase(flat, pattern) for pattern in patterns or live_patterns())


def _nested(values):
    """Adapt flat node parameters to the domain compiler's ROS YAML shape."""
    result = {}
    for key, value in values.items():
        node, param = split(key)
        section = result.setdefault(node, {}).setdefault('ros__parameters', {})
        parts = param.split('.')
        for part in parts[:-1]:
            section = section.setdefault(part, {})
        section[parts[-1]] = value
    return result


def _flat(config):
    result = {}

    def visit(section, prefix):
        for key, value in section.items():
            name = prefix + '.' + key
            if isinstance(value, dict):
                visit(value, name)
            else:
                result[name] = value

    for node, section in config.items():
        visit(section['ros__parameters'], node)
    return result


def derived(values, floor=None):
    """Compile flat preview values through Nav2's canonical pure derivation.

    Plugin-free FollowPath fixtures retain the domain helper's explicit legacy
    support. Missing inputs stay missing; the selected base owns all defaults.
    """
    from mhseals_nav import navigation_config

    config = _nested(values)
    original = _nested(values)
    controller = original.get('controller_server', {}).get('ros__parameters', {})
    name = navigation_config.mppi_controller(controller, legacy=True)
    if name is not None and floor is not None:
        controller[name]['prune_distance'] = floor
    return _flat(navigation_config.derived(config, original))


def plan(changes, current, declared=None, patterns=None):
    """Group a change set by how it is applied.

    ``changes`` and ``current`` are flat dicts (current: the live values read back); ``declared``
    maps a node to the parameter names it declares (None: unknown). Derived keys the change moves
    are added. Returns ``{'live': {node: {param: value}}, 'reset': ..., 'missing': [key, ...],
    'unchanged': [key, ...]}``; nodes follow ORDER. Unchanged values are dropped so a full param
    load stays harmless; a key its node does not declare cannot be set at all (``missing``).
    """
    patterns = patterns or live_patterns()
    from mhseals_nav import navigation_config

    # Use the same explicit-derived conflict check as full launch compilation.
    # This planner is a pure preview helper, not a runtime apply route.
    base, explicit = _nested(current), _nested(changes)
    controller = base.get('controller_server', {}).get('ros__parameters', {})
    # Complete plugin-aware requests use the launch validation contract. Only
    # historical plugin-free fixtures retain the partial-helper compatibility.
    plugin_aware = 'controller_plugins' in controller or any(
        isinstance(value, dict) and 'plugin' in value for value in controller.values())
    # The launch ceilings bound a live request too: beyond one is refused, as at launch.
    ceilings = _ceilings()

    def compile_values(base, explicit):
        return navigation_config.lower_navigation(
            base, navigation_config.within_ceilings(ceilings, explicit, strict=True))
    if plugin_aware:
        navigation_config.validate_navigation_base(base)

        def compile_values(base, explicit):
            return navigation_config.validated_merge_limits(base, explicit, ceilings)
    before = _flat(compile_values(base, {}))
    after = _flat(compile_values(base, explicit))
    # Requested values may be capped or normalized by the compiler too.
    wanted = {key: after['.'.join(split(key))] for key in changes}
    requested = {'.'.join(split(key)) for key in changes}
    wanted.update({k: v for k, v in after.items() if before.get(k) != v and k not in requested})
    groups = {'live': {}, 'reset': {}}
    unchanged, missing = [], []
    for key, value in wanted.items():
        old = current.get(key)
        if old == value:
            unchanged.append(key)
            continue
        node, param = split(key)
        names = (declared or {}).get(node)
        if names is not None and param not in names:
            missing.append(key)
            continue
        cls = 'live' if is_live(key, patterns) else 'reset'
        groups[cls].setdefault(node, {})[param] = value
    for cls in groups:
        groups[cls] = dict(sorted(groups[cls].items(), key=lambda item: rank(item[0])))
    return {**groups, 'missing': missing, 'unchanged': unchanged}
