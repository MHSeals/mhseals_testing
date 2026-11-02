"""Sensor fusion: what the EKFs fuse, camera-lidar calibration and the live fusion feed.

    ekf_params  what robot_localization fuses, read from an EKF node's parameters
    calib       camera-lidar extrinsic calibration
    campaign    calibration campaigns over recorded runs
    ros_feed    the live fusion feed behind the Fusion panel
"""

from importlib import import_module

# name -> submodule; resolved on first use, so importing the facade loads nothing (some submodules need rclpy)
_EXPORTS = {
    'FusionFeed': 'ros_feed',
    'StaticTF': 'ros_feed',
    'detections': 'ros_feed',
    'load_config': 'ros_feed',
    'id_changes': 'campaign',
}
__all__ = sorted(_EXPORTS)


def __getattr__(name):
    if name not in _EXPORTS:
        raise AttributeError(f'module {__name__!r} has no attribute {name!r}')
    value = getattr(import_module(f'{__name__}.{_EXPORTS[name]}'), name)
    globals()[name] = value
    return value
