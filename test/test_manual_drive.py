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


class Clocked(Keys):
    """Keys on a fake clock: every clock read costs 0.1 ms (cost), a blocking read waits its timeout."""

    def __init__(self, batches):
        super().__init__(batches)
        self.now, self.stall, self.cost = 0.0, 0.0, 1e-4

    def clock(self):
        self.now += self.cost
        return self.now

    def read(self, timeout=None):
        key = super().read(timeout)
        if timeout:
            self.now += timeout + self.stall
            self.stall = 0.0
        return key


def test_the_drain_is_bounded_from_the_last_publish_not_from_the_batch_start():
    keys = Clocked([['press:w'] + ['repeat:w'] * 5000])
    left, times = [], []

    def show(*args):
        times.append(keys.now)
        left.append(len(keys.batches[0]))
        keys.batches = [['x']]       # leave after the first publish
    drive(keys, ManualState(), Publisher(), show, clock=keys.clock)
    assert 0.09 < times[0] < 0.11    # ~DRAIN_S after the start, not 50 ms wait + 100 ms
    assert left[0] > 0               # cut by time: the rest stays queued


def test_a_stall_past_the_budget_still_reads_the_stale_burst_whole():
    keys = Clocked([['press:w'] + ['repeat:w'] * 200 + ['release:w']])
    keys.cost = 1e-6                 # a real read of a queued event is microseconds
    keys.stall = 30.0                # the link stalls 30 s before the burst arrives
    publisher = Publisher()
    drive(keys, ManualState(), publisher, lambda *args: None, clock=keys.clock)
    assert publisher.sent[0] == (0.0, 0.0, 0.0)
