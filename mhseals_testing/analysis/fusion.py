"""What robot_localization fuses, read from an EKF node's parameters (docs: nav note, fusion table).

Pure: the caller fetches the parameters (``GetParameters`` on /ekf_local, /ekf_global) and passes
a plain dict, so this runs and is tested without ROS.
"""
import math
from dataclasses import dataclass

# robot_localization's 15-element mask order.
STATES = ('x', 'y', 'z', 'roll', 'pitch', 'yaw', 'vx', 'vy', 'vz',
          'vroll', 'vpitch', 'vyaw', 'ax', 'ay', 'az')
KINDS = ('odom', 'imu', 'pose', 'twist')


@dataclass(frozen=True)
class Input:
    name: str      # odom0, imu0, ...
    topic: str
    fused: tuple   # state names the mask turns on, in STATES order


def inputs_from_params(params):
    """The inputs an EKF declares: ``<kind>N`` topics with their ``<kind>N_config`` masks.

    An input without a topic is skipped; a missing or malformed mask fuses nothing (the node
    would refuse it, and the popup must show that rather than guess).
    """
    found = []
    for kind in KINDS:
        index = 0
        while f'{kind}{index}' in params:
            name = f'{kind}{index}'
            mask = params.get(f'{name}_config') or ()
            fused = tuple(state for state, on in zip(STATES, mask) if on) if len(mask) == len(STATES) else ()
            if params[name]:
                found.append(Input(name, str(params[name]), fused))
            index += 1
    return found


def trust(sigmas):
    """Each source's share of one state, ``(1/s_i^2) / sum(1/s_j^2)``, from ``{input: sigma}``.

    The measurement weight is only meaningful against another source, so fewer than two usable
    sigmas (positive and finite) return ``{}``; a lone source is simply the whole estimate.
    """
    weights = {name: 1.0 / (sigma * sigma) for name, sigma in sigmas.items()
               if math.isfinite(sigma) and sigma > 0}
    total = sum(weights.values())
    if len(weights) < 2 or not math.isfinite(total):
        return {}
    return {name: weight / total for name, weight in weights.items()}
