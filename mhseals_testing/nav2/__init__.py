"""Nav2: the live client, parameter edits and the live apply.

    ros_client  the thin Nav2 client on the RosIO node (lifecycle, goals, costmap, map pose)
    live        the ROS-free live apply sequence behind the TUI popup and boat_trial config apply nav2
    ros_live    the ROS ports of that apply (parameter clients, mux, authority, rosout)
    edit        the user Nav2 parameter file and its edits
    params      the parameter classes (config/nav2_classes.yaml)
    limits      the characterized speed limits Nav2 loads
"""

from importlib import import_module

# name -> submodule; resolved on first use, so importing the facade loads nothing (some submodules need rclpy)
_EXPORTS = {
    'Nav2': 'ros_client',
    'wait': 'ros_client',
    'Nav2Live': 'live',
    'Ports': 'live',
    'RosPorts': 'ros_live',
    'Rosout': 'ros_live',
    'cli_node': 'ros_live',
    'session_journal': 'ros_live',
    'Nav2File': 'edit',
}
__all__ = sorted(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    value = getattr(import_module(f'{__name__}.{_EXPORTS[name]}'), name)
    globals()[name] = value
    return value
