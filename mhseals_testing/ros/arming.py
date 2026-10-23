"""Arming: run the thruster driver (and the velocity controller) as supervised children.

The driver is mhseals_hardware's thruster_pwm_node, configured from a snapshot
of the session's hardware config, so the report records exactly what drove the
thrusters. Disarming stops the children; the driver neutralizes its outputs
(and cuts ESC power on the boat) as it exits.
"""

import time

from mhseals_hardware.configuration import save_config

DRIVER = 'thruster_pwm_node'
EFFORT_TOPIC = '/cmd_vel_effort'


def driver_commands(profile, config_path):
    """(name, argv) for each child, the driver first so it holds neutral before commands."""
    common = ['-p', f'config_file:={config_path}',
              '-p', f'use_sim_time:={str(profile.use_sim_time).lower()}']
    driver = ['ros2', 'run', 'mhseals_hardware', DRIVER, '--ros-args', *common,
              '-p', f'output_backend:={profile.outputs}']
    if not profile.velocity_controller:
        return [('hardware', driver + ['-r', f'cmd_vel:={profile.command_topic}'])]
    controller = ['ros2', 'run', 'mhseals_hardware', 'calibrated_velocity_controller',
                  '--ros-args', *common,
                  '-p', f'input_topic:={profile.command_topic}',
                  '-p', f'output_topic:={EFFORT_TOPIC}',
                  '-p', f'odom_topic:={profile.odometry}']
    return [('hardware', driver + ['-r', f'cmd_vel:={EFFORT_TOPIC}']),
            ('velocity', controller)]


class Arming:
    def __init__(self, profile, ros, processes, hardware_config, config_path,
                 clock=time.monotonic, sleep=time.sleep):
        self.profile = profile
        self.ros = ros
        self.processes = processes
        self.hardware_config = hardware_config
        self.config_path = config_path
        self.clock = clock
        self.sleep = sleep
        self.armed = False
        self.pending = None

    @property
    def driver_topic(self):
        return EFFORT_TOPIC if self.profile.velocity_controller else self.profile.command_topic

    def children(self):
        return [name for name, _ in driver_commands(self.profile, self.config_path)]

    def fault(self):
        """Why the armed outputs can't be trusted, or ''."""
        if not self.armed or self.profile.outputs == 'none':
            return ''
        exited = [name for name in self.children() if not self.processes.running(name)]
        return f"{', '.join(exited)} exited; disarm and check its log" if exited else ''

    def begin(self):
        """Start the driver without waiting; ``poll`` reports when it listens."""
        if self.armed or self.pending is not None:
            return
        if self.profile.outputs == 'none':  # the plant takes commands directly
            self.armed = True
            return
        if any(name.endswith(DRIVER) for name in self.ros.subscribers(self.driver_topic)):
            raise RuntimeError(f'another {DRIVER} already drives {self.driver_topic}')
        save_config(dict(self.hardware_config, output_backend=self.profile.outputs),
                    self.config_path)
        for name, argv in driver_commands(self.profile, self.config_path):
            self.processes.start(name, argv)
        self.pending = self.clock()

    def poll(self, timeout=10.0):
        """'armed' once the driver subscribes, 'starting' until then; raises on failure."""
        if self.armed:
            return 'armed'
        if self.pending is None:
            raise RuntimeError('arming was not started')
        if any(name.endswith(DRIVER) for name in self.ros.subscribers(self.driver_topic)):
            self.armed, self.pending = True, None
            return 'armed'
        exited = [name for name in self.children() if not self.processes.running(name)]
        if exited or self.clock() - self.pending > timeout:
            self.cancel()
            raise RuntimeError(f"{', '.join(exited)} exited during startup" if exited
                               else f'{DRIVER} did not subscribe to {self.driver_topic}')
        return 'starting'

    def cancel(self):
        """Abandon a start in progress: the driver is stopped, nothing is armed."""
        self.pending = None
        self.stop_children()

    def arm(self, timeout=10.0):
        """Blocking arm, for callers without a view to keep drawing."""
        self.begin()
        while self.poll(timeout) == 'starting':
            self.sleep(0.1)

    def stop_children(self):
        for name in reversed(self.children()):  # controller first, then the driver
            self.processes.stop(name)

    def disarm(self):
        self.armed, self.pending = False, None
        if self.profile.outputs != 'none':
            self.stop_children()
