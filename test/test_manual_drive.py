"""Manual drive reads a whole queued batch before publishing (no stale-repeat pulse after a stall)."""

from mhseals_hardware.manual_state import ManualState

from mhseals_testing.procedures.manual import drive


class Keys:
    def __init__(self, batches):
        self.batches = [list(batch) for batch in batches]

    def read(self, timeout=None):
        if timeout == 0:   # what is already queued
            return self.batches[0].pop(0) if self.batches and self.batches[0] else None
        while self.batches and not self.batches[0]:
            self.batches.pop(0)
        return self.batches[0].pop(0) if self.batches else 'x'   # then leave


class Publisher:
    def __init__(self):
        self.sent = []

    def publish(self, command):
        self.sent.append(command)

    def hold(self):
        pass


def test_a_burst_of_stale_repeats_meets_its_release_before_anything_is_published():
    # A link stall delivers a held key's repeats and its release in one burst, more than 64 events.
    burst = ['press:w'] + ['repeat:w'] * 200 + ['release:w']
    publisher = Publisher()
    drive(Keys([burst]), ManualState(), publisher, lambda *args: None, clock=lambda: 1.0)
    assert publisher.sent == [(0.0, 0.0, 0.0)]


def test_an_unfinished_batch_is_carried_over_not_dropped():
    times = iter(range(10 ** 6))   # every clock read is 1 s later: the drain runs out at once
    publisher = Publisher()
    drive(Keys([['press:w', 'release:w']]), ManualState(), publisher, lambda *args: None,
          clock=lambda: next(times) * 1e-3)
    assert publisher.sent[-1] == (0.0, 0.0, 0.0)   # the release was not lost
