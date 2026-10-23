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
# velocity_smoother list -> (MPPI limit per axis, sign), as mhseals_nav.navigation_config.derived.
SMOOTHER = {'max_velocity': (('vx_max', 1), ('vy_max', 1), ('wz_max', 1)),
            'min_velocity': (('vx_min', 1), ('vy_max', -1), ('wz_max', -1)),
            'max_accel': (('ax_max', 1), ('ay_max', 1), ('az_max', 1)),
            'max_decel': (('ax_min', 1), ('ay_max', -1), ('az_max', -1))}
# rotation shim key -> the MPPI limit it follows.
SHIM = (('rotate_to_heading_angular_vel', 'wz_max'), ('max_angular_accel', 'az_max'))
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


def derived(values, floor=None):
    """The keys that follow from ``values`` (flat): model_dt, prune_distance, std clamps, shim rates, smoother limits.

    ``floor`` is the current prune_distance, which only ever rises (the same rule as
    mhseals_nav.navigation_config.derived, which this mirrors until it is shared).
    """
    out = {}
    freq = values.get('controller_server.controller_frequency')
    if freq:
        out[FOLLOW + 'model_dt'] = round(1 / freq, 6)
    follow = {k[len(FOLLOW):]: v for k, v in values.items() if k.startswith(FOLLOW)}
    follow['model_dt'] = out.get(FOLLOW + 'model_dt', follow.get('model_dt', .05))
    for axis in ('vx', 'vy', 'wz'):  # sampling noise above an axis limit would clip most samples
        if f'{axis}_std' in follow and f'{axis}_max' in follow:
            out[FOLLOW + f'{axis}_std'] = min(follow[f'{axis}_std'], follow[f'{axis}_max'])
    # The rotation shim turns no faster than MPPI is held to.
    for shim, limit in SHIM:
        if shim in follow and limit in follow:
            out[FOLLOW + shim] = follow[limit]
    if 'vx_max' in follow:
        out[FOLLOW + 'prune_distance'] = max(floor or 0., round(
            1.2 * follow['vx_max'] * follow.get('time_steps', 56) * follow['model_dt'], 2))
    for name, axes in SMOOTHER.items():
        current = values.get('velocity_smoother.' + name)
        if current is not None and all(limit in follow for limit, _ in axes):
            out['velocity_smoother.' + name] = [sign * follow[limit] for limit, sign in axes]
    return out


def plan(changes, current, declared=None, rules=None):
    """Group a change set by class and node.

    ``changes`` and ``current`` are flat dicts (current: the live values read back). Derived
    keys the change moves are added. Returns ``{'live': {node: {param: value}}, 'reset': ...,
    'restart': ..., 'unchanged': [key, ...]}``; nodes follow ORDER. Unchanged values are
    dropped so a full param load stays harmless.
    """
    rules = rules or load_rules()
    wanted = dict(changes)
    floor = current.get(FOLLOW + 'prune_distance')
    before, after = derived(current, floor), derived({**current, **changes}, floor)
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
