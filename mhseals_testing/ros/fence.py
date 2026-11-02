"""fence_watchdog's live course area from the TUI: course.corners and on_exit (SetParameters, read back) and
/fence/anchor (std_srvs/Trigger). Calls block for a few seconds at most; the TUI runs them off the key loop.
The mhseals_nav fence contract: see harness/course_area.py.
"""

from types import SimpleNamespace

from mhseals_testing.harness import course_area
from mhseals_testing.ros.params import Params

NODE = '/fence_watchdog'
ANCHOR = '/fence/anchor'


class FenceClient:
    def __init__(self, node, params=None):
        self.node = node
        self.params = params or Params(SimpleNamespace(node=node))
        self._anchor = None

    def set_corners(self, corners):
        """Apply ``corners`` [(lat, lon)] whole, then read them back: ``(ok, why)``."""
        flat = course_area.flatten(corners)
        ok, reason = self.params.set_atomically(NODE, {'course.corners': flat})
        if not ok:
            return False, reason or 'fence_watchdog refused the corners'
        back = course_area.pairs(list(self.params.get(NODE, ['course.corners']).get('course.corners') or []))
        if not course_area.same(back, corners):
            return False, f'readback differs: fence_watchdog holds {len(back)} corners'
        return True, ''

    def set_on_exit(self, mode):
        """``on_exit`` to alert or return, read back: ``(ok, why)``."""
        if mode not in course_area.ON_EXIT:
            return False, f'on_exit is alert or return, not {mode!r}'
        ok, reason = self.params.set_atomically(NODE, {'on_exit': mode})
        if not ok:
            return False, reason or 'refused'
        back = self.params.get(NODE, ['on_exit']).get('on_exit')
        return (back == mode), ('' if back == mode else f'readback says {back!r}')

    def anchor(self, timeout=3.0):
        """/fence/anchor: ``(success, message)``; a lat/lon course area answers "course area fixed"."""
        from std_srvs.srv import Trigger
        from mhseals_testing.ros.nav2 import wait
        if self._anchor is None:
            self._anchor = self.node.create_client(Trigger, ANCHOR)
        if not self._anchor.wait_for_service(timeout_sec=timeout):
            return False, f'{ANCHOR} is not available'
        reply = wait(self._anchor.call_async(Trigger.Request()), timeout, 'fence anchor')
        return bool(reply.success), str(reply.message)
