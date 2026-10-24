"""Read-only TopicPWMOutputs observation on the configured hardware pwm_topic."""

import threading
import time

from mhseals_hardware.thruster_mixer import MAX_PWM, MIN_PWM
from mhseals_testing.health.pwm import Output, Readback


class TopicReadback:
    """One subscription, shared immutable observations, no polling or publishing.

    Construct only for the topic backend. Configuration/status topics do not
    provide pulse evidence. Closing invalidates the cache and destroys the sub.
    """

    def __init__(self, node, topic, *, clock=time.monotonic, message_type=None):
        if not isinstance(topic, str) or not topic.strip():
            raise ValueError('configured pwm_topic is required')
        if message_type is None:
            from std_msgs.msg import UInt16MultiArray
            message_type = UInt16MultiArray
        self._node, self._clock, self.topic = node, clock, topic
        self._lock = threading.Lock()
        self._closed = False
        self._observation = Readback('topic', None)
        self._subscription = node.create_subscription(message_type, topic, self._receive, 10)

    def _receive(self, message):
        with self._lock:
            if self._closed:
                return
            receipt = self._clock()
            pulses = getattr(message, 'data', ())
            try:
                valid = len(pulses) == 4 and all(
                    type(pulse) is int and MIN_PWM <= pulse <= MAX_PWM for pulse in pulses)
            except TypeError:
                valid = False
            outputs = tuple(Output(f'{self.topic}#{index}', 'pulse', duty_ns=pulse * 1000)
                            for index, pulse in enumerate(pulses, 1)) if valid else ()
            # Invalid messages replace prior good evidence immediately.
            self._observation = Readback('topic', receipt, outputs, True, valid)

    def observation(self):
        with self._lock:
            return self._observation

    def close(self):
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._observation = Readback('topic', None)
        self._node.destroy_subscription(self._subscription)
