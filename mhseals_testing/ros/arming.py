"""Arming: run the thruster driver (and the velocity controller) as supervised children.

The driver is mhseals_hardware's thruster_pwm_node, configured from a snapshot
of the session's hardware config, so the report records exactly what drove the
thrusters. Disarming stops the children; the driver neutralizes its outputs
(and cuts ESC power on the boat) as it exits.
"""

import time

from mhseals_hardware.configuration import save_config
from mhseals_hardware.velocity_law import Plant

from mhseals_testing.profiles import MUX_INPUT  # noqa: F401  (re-exported for the authority feed)

DRIVER = 'thruster_pwm_node'
CONTROLLER = 'calibrated_velocity_controller'
EFFORT_TOPIC = '/cmd_vel_effort'
MUX = 'command_mux'
APPLIED_TOPIC = '/cmd_vel/applied'  # what the mux lets through to the driver


def muxed(profile):
    """True when a command_mux child arbitrates the profile's command topic."""
    return profile.muxed


def command_input(profile):
    """The topic the first child after the mux (driver or controller) listens on."""
    return APPLIED_TOPIC if muxed(profile) else profile.command_topic


def controller_topics(profile):
    """The topics the profile binds the velocity controller to."""
    if not getattr(profile, 'velocity_controller', False):
        return {}
    return dict(input_topic=command_input(profile), output_topic=EFFORT_TOPIC,
                odom_topic=profile.odometry)


def driver_commands(profile, config_path, topics=None):
    """(name, argv) for each child, the driver first so it holds neutral before commands, the mux last."""
    if topics is None:
        topics = controller_topics(profile)
    common = ['-p', f'config_file:={config_path}',
              '-p', f'use_sim_time:={str(profile.use_sim_time).lower()}']
    mux_common = list(common)
    driver = ['ros2', 'run', 'mhseals_hardware', DRIVER, '--ros-args', *common,
              '-p', f'output_backend:={profile.outputs}']
    mux = [('mux', ['ros2', 'run', 'mhseals_hardware', MUX, '--ros-args', *mux_common])]
    mux = mux if muxed(profile) else []
    if not profile.velocity_controller:
        return [('hardware', driver + ['-r', f'cmd_vel:={command_input(profile)}']), *mux]
    controller = ['ros2', 'run', 'mhseals_hardware', 'calibrated_velocity_controller',
                  '--ros-args', *common,
                  *[part for key, value in topics.items() for part in ('-p', f'{key}:={value}')]]
    return [('hardware', driver + ['-r', f"cmd_vel:={topics['output_topic']}"]),
            ('velocity', controller), *mux]


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
        self.topics = None

    @property
    def driver_topic(self):
        return (self.bindings()['output_topic'] if self.profile.velocity_controller
                else command_input(self.profile))

    def bindings(self):
        if self.topics is not None:
            return self.topics
        return controller_topics(self.profile)

    def children(self):
        return [name for name, _ in driver_commands(
            self.profile, self.config_path, self.bindings())]

    def fault(self):
        """Why the armed outputs can't be trusted, or ''."""
        if not self.armed or self.profile.outputs == 'none':
            return ''
        exited = [name for name in self.children() if not self.processes.running(name)]
        return f"{', '.join(exited)} exited; disarm and check its log" if exited else ''

    def rival(self):
        """Why arming would be refused because another driver owns the topic, or ''."""
        if any(name.endswith(DRIVER) for name in self.ros.subscribers(self.driver_topic)):
            return f'another {DRIVER} already drives {self.driver_topic}'
        # A driver on another topic (the sim rig's own) would still write the same outputs.
        if any(name.rsplit('/', 1)[-1] == DRIVER for name in self.ros.nodes()):
            return f'another {DRIVER} node is on the graph; it owns the outputs'
        if muxed(self.profile) and self.ros.publishers(APPLIED_TOPIC):
            return f'{APPLIED_TOPIC} already has a publisher (another {MUX}?); stop it first'
        return ''

    def begin(self):
        """Start the driver without waiting; ``poll`` reports when it listens."""
        if self.armed or self.pending is not None:
            return
        self.topics = controller_topics(self.profile)
        if self.profile.outputs == 'none':  # the plant takes commands directly
            self.armed = True
            return
        rival = self.rival()
        if rival:
            raise RuntimeError(rival)
        if self.profile.velocity_controller:  # fail now, not after the 10 s timeout
            try:
                Plant(self.hardware_config)
            except (ValueError, KeyError, TypeError) as error:  # partial gain rows
                raise RuntimeError(f'velocity model invalid: {error}') from error
        save_config(dict(self.hardware_config, output_backend=self.profile.outputs),
                    self.config_path)
        for name, argv in driver_commands(
                self.profile, self.config_path, self.bindings()):
            self.processes.start(name, argv)
        self.pending = self.clock()

    def expected(self):
        """The topic each child must subscribe to before arming completes."""
        topics = {DRIVER: self.driver_topic}
        if self.profile.velocity_controller:
            topics[CONTROLLER] = self.bindings()['input_topic']
        return topics

    def listening(self):
        """What is not yet subscribed: the driver, and the controller when one is used.

        The controller subscribes only after its velocity model validated, so its
        subscription doubles as the readiness signal.
        """
        return [name for name, topic in self.expected().items()
                if not any(sub.endswith(name) for sub in self.ros.subscribers(topic))]

    def startup_failure(self, missing, exited):
        """Say which of three things went wrong while the children were starting."""
        if exited:
            last = self.processes.last_line(exited[0])
            return f"{', '.join(exited)} exited during startup" + (f': {last}' if last else '')
        nodes = self.ros.nodes()
        if not any(node.endswith(f'/{missing[0]}') for node in nodes):
            return (f'{missing[0]} is running but not discovered; check ROS_DOMAIN_ID, '
                    'RMW_IMPLEMENTATION and the DDS profile')
        if not self.ros.parameter_service(missing[0]):
            return f'{missing[0]} is discovered but its parameter service is down'
        return f'{missing[0]} did not subscribe to {self.expected()[missing[0]]}'

    def poll(self, timeout=10.0):
        """'armed' once everything listens, 'starting' until then; raises on failure."""
        if self.armed:
            return 'armed'
        if self.pending is None:
            raise RuntimeError('arming was not started')
        missing = self.listening()
        if not missing:
            self.armed, self.pending = True, None
            return 'armed'
        exited = [name for name in self.children() if not self.processes.running(name)]
        if exited or self.clock() - self.pending > timeout:
            reason = self.startup_failure(missing, exited)
            self.cancel()
            raise RuntimeError(reason)
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

    def restart_driver(self, config, confirm):
        """Run only the driver from ``config`` (the saved operator file) while ``confirm()``
        reads its outputs back, then stop it again: disarmed means no driver holds the ESCs
        powered. The caller checks that it is disarmed."""
        rival = self.rival()
        if rival:
            raise RuntimeError(rival)
        save_config(dict(config, output_backend=self.profile.outputs), self.config_path)
        children = driver_commands(self.profile, self.config_path, self.bindings())
        try:
            self.processes.start('hardware', dict(children)['hardware'])
            return confirm()
        finally:
            self.processes.stop('hardware')   # outputs disabled, ESC power off

    def stop_children(self):
        for name in reversed(self.children()):  # controller first, then the driver
            self.processes.stop(name)

    def disarm(self):
        self.armed, self.pending = False, None
        if self.profile.outputs != 'none':
            self.stop_children()
