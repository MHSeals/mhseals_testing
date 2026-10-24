"""What a simulator session records beyond a boat session (bags.md B14). ROS-free.

* the profile's extra bag topics (not passed to ros2 bag in full mode) (``extra:`` in config/bags/topics.yaml: ``/clock`` and the truth odometry);
* the clock domain in ``session.json`` (JSON only, data_layout), so readers know bag stamps are wall time while
  ``ros_time`` in the journal is sim time;
* a copy of the profile's course truth next to the session, so the run can be judged after the
  simulator's own file is overwritten;
* a rewind detector for ``/clock``: an episode reset makes sim time jump back.
"""
import shutil
import time
from pathlib import Path

import yaml

from mhseals_testing import data

TOPICS = Path(__file__).resolve().parents[2] / 'config' / 'bags' / 'topics.yaml'
REWIND_S = 0.5  # a backwards step smaller than this is jitter, not an episode reset


def extra_topics(profile_name, path=TOPICS):
    """Topics recorded in addition to the common filter for this profile (empty for the boat)."""
    config = yaml.safe_load(Path(path).read_text()) or {}
    return list(((config.get('extra') or {}).get(profile_name) or {}).get('topics') or [])


def session_record(profile, session_id):
    """The identity and clock-domain fields start_session merges into ``session.json``."""
    sim = bool(profile.use_sim_time)
    return {'id': session_id, 'kind': 'sim' if sim else 'boat',
            'profile': profile.name, 'ground_truth': profile.ground_truth,
            'clock': {'domain': 'sim' if sim else 'wall', 'time_scale': 1.0}}


def copy_course_truth(source, destination):
    """Copy the simulator's course truth into the session; False (not an error) when it is not there yet."""
    if not source or not Path(source).is_file():
        return False
    shutil.copyfile(source, destination)
    return True


def start_session(profile, session_path, truth_path):
    """Merge the clock domain into ``session.json`` (and copy the truth for a simulator); returns the record."""
    session_path = Path(session_path)
    record = session_record(profile, session_path.parent.name)
    if profile.use_sim_time:
        record['course_truth'] = Path(truth_path).name if copy_course_truth(
            profile.course_truth, truth_path) else None
    header = data.read_json(session_path)
    data.write_json(session_path, {**(header if isinstance(header, dict) else {}), **record}, 'mhseals.session/1')
    return record


class ClockWatch:
    """Reports each backwards step of the simulator clock once."""

    def __init__(self, threshold=REWIND_S):
        self.threshold, self.last = threshold, None

    def feed(self, seconds):
        """True when ``seconds`` is earlier than the last reading by more than the threshold."""
        previous, self.last = self.last, seconds
        return previous is not None and seconds < previous - self.threshold
