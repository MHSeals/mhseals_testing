"""Manual control through mhseals_hardware's deadman keyboard driver, under the interlocks."""

from mhseals_testing.runner import Aborted


class GuardedPublisher:
    """keyboard_control publishes Twists; non-neutral ones must pass the interlocks.

    Neutral is always allowed so the driver's final stop goes out even after a
    blocker trips.
    """

    def __init__(self, plant, blockers):
        self.plant = plant
        self.blockers = blockers
        self.command = (0.0, 0.0, 0.0)

    def publish(self, message):
        command = (message.linear.x, message.linear.y, message.angular.z)
        if any(command):
            reasons = self.blockers()
            if reasons:
                self.plant.neutral()
                raise Aborted('interlock: ' + '; '.join(reasons))
        self.command = command
        self.plant.publish(command)


def run(ctx):
    from mhseals_hardware.keyboard_control import run_manual  # imports rclpy
    manual = ctx.services['manual']
    publisher = GuardedPublisher(manual['plant'], manual['blockers'])
    config = ctx.services['hardware_config']
    try:
        # keyboard_control needs key-release events, so it takes the terminal meanwhile.
        with manual['suspended']():
            run_manual(publisher, manual['console'], config['manual_amplitude'],
                       config['manual_repeat_timeout'], config['manual_initial_timeout'],
                       status_panel=manual['status'])
    finally:
        manual['plant'].neutral()
