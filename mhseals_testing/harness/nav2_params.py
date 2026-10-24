"""Which Nav2 parameters can change on a running stack, and in what order. Pure, ROS-free.

Keys are flat ``<node>.<param>`` strings (``controller_server.FollowPath.vx_max``); the two
costmaps are ``local_costmap`` / ``global_costmap`` and may repeat the name as in the YAML
(``local_costmap.local_costmap.footprint``). The class table is config/nav2_classes.yaml.
Spec: authority_hotreload 7.3 (HR6). Applying a plan is Nav2Apply (HR7).
"""

from fnmatch import fnmatchcase
from pathlib import Path

import yaml

CLASSES = ('live', 'reset', 'restart')
# Apply order for the live group; the smoother is also sent first, at the more restrictive limits.
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


def load_rules(path=None):
    """The ordered (node, param, class) rules; a bad class is an error at load, not at apply."""
    rules = yaml.safe_load(Path(path or _rules_path()).read_text())['rules']
    for rule in rules:
        if rule['class'] not in CLASSES:
            raise ValueError(f"nav2_classes: {rule['node']}.{rule['param']}: unknown class {rule['class']!r}")
    return [(r['node'], r['param'], r['class']) for r in rules]


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


def classify(key, old=None, new=None, declared=None, rules=None):
    """'live', 'reset' or 'restart' for changing ``key`` from ``old`` to ``new``.

    ``declared`` is the node's parameter names from list_parameters (None: unknown, so the
    undeclared rule is skipped). A name the node never declared (a new critic, layer or
    source) needs a restart. A sign change of vx_min is a reset: MPPI reads it at configure.
    """
    node, param = split(key)
    if declared is not None and param not in declared:
        return 'restart'
    if key == FOLLOW + 'vx_min' and old is not None and new is not None and (old < 0) != (new < 0):
        return 'reset'
    for rule_node, rule_param, cls in rules or load_rules():
        if fnmatchcase(node, rule_node) and fnmatchcase(param, rule_param):
            return cls
    return 'reset'


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


def plan(changes, current, declared=None, rules=None):
    """Group a change set by class and node.

    ``changes`` and ``current`` are flat dicts (current: the live values read back). Derived
    keys the change moves are added. Returns ``{'live': {node: {param: value}}, 'reset': ...,
    'restart': ..., 'unchanged': [key, ...]}``; nodes follow ORDER. Unchanged values are
    dropped so a full param load stays harmless.
    """
    rules = rules or load_rules()
    wanted = dict(changes)
    from mhseals_nav import navigation_config

    # Use the same explicit-derived conflict check as full launch compilation.
    # This planner is a pure preview helper, not a runtime apply route.
    base, explicit = _nested(current), _nested(changes)
    controller = base.get('controller_server', {}).get('ros__parameters', {})
    # Complete plugin-aware requests use the launch validation contract. Only
    # historical plugin-free fixtures retain the partial-helper compatibility.
    plugin_aware = 'controller_plugins' in controller or any(
        isinstance(value, dict) and 'plugin' in value for value in controller.values())
    compile_values = navigation_config.lower_navigation
    if plugin_aware:
        navigation_config.validate_navigation_base(base)
        compile_values = navigation_config.validated_merge_limits
    before = _flat(compile_values(base, {}))
    after = _flat(compile_values(base, explicit))
    wanted.update({k: v for k, v in after.items() if before.get(k) != v and k not in changes})
    groups = {cls: {} for cls in CLASSES}
    unchanged = []
    for key, value in wanted.items():
        old = current.get(key)
        if old == value:
            unchanged.append(key)
            continue
        node, param = split(key)
        cls = classify(key, old, value, None if declared is None else declared.get(node), rules)
        groups[cls].setdefault(node, {})[param] = value
    rank = {node: i for i, node in enumerate(ORDER)}
    for cls in CLASSES:
        groups[cls] = dict(sorted(groups[cls].items(), key=lambda item: rank.get(item[0], len(ORDER))))
    return {**groups, 'unchanged': unchanged}
