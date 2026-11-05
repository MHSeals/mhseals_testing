"""Health advisories and the health engine: the Status rows' advisory samples, facts and snapshots."""

from mhseals_testing.health import legacy as health
from mhseals_testing.profiles import drives_pwm_pins
from mhseals_testing.ros.arming import EFFORT_TOPIC, command_input


ADVISORY_PERIOD = 1.0


class Health:
    """Health advisories and the health engine, mixed into Settings."""

    def refresh_advisories(self):
        """Sample the non-blocking checks at most once per ADVISORY_PERIOD."""
        if self.clock() - self.state.health.last_advisory < ADVISORY_PERIOD:
            return
        self.state.health.last_advisory = self.clock()
        seen, live = self.foreign_instances()
        self.feed_map()
        self.monitor.advise('instances', 'FAIL' if live else 'WARN' if seen else 'OK',
                            ', '.join(seen) if seen else 'no other instance')
        self.sync_guests()
        snapshot = self.snapshot()
        for name, (level, detail) in self.health.tick(snapshot).items():
            self.monitor.advise(name, level, detail)
        if self.health.error:
            self.monitor.advise('health config', 'FAIL', self.health.error)
        if self.profile.use_sim_time and self.ros.now() == 0:
            # Without /clock every stamp looks stale; say why instead.
            self.monitor.advise('clock', 'FAIL', 'use_sim_time but no /clock received')
        network = self.network.poll() if self.network else None
        if network:
            self.monitor.advise('network', *network)
        if getattr(self.network, 'clocks', None):       # pre.clock_sync judges clock.spread
            self.health.facts.publish('clock_sync', self.network.clock_facts)
        self.monitor.advise('recording', 'OK' if self.processes.running('bag') else 'FAIL',
                            str(self.paths.bag))
        self.watch_recording()
        exited = [name for name in self.processes.exited() if name != 'bag']
        if exited:
            self.monitor.advise('processes', 'WARN', 'exited: ' + ', '.join(exited))
        if self.profile.outputs != 'none':
            topic = EFFORT_TOPIC if self.profile.velocity_controller else command_input(self.profile)
            self.monitor.advise('PWM driver', *health.pwm_driver(
                self.ros.nodes(), self.ros.subscribers(topic), topic,
                self.armed, self.processes.running('hardware')))
        if drives_pwm_pins(self.profile):   # the sysfs advisory always: enables and disarmed-but-on
            config = self.args.hardware_config
            level, detail, self.state.health.pwm_sysfs = health.pwm_readback(
                config['pwm_chips'], config['pwm_channels'], self.armed, config['frequency'])
            self.monitor.advise('PWM readback', level, detail)
            self.state.health.pwm_readback = list(self.state.health.pwm_sysfs)

    def show_driver_readback(self, pulses):
        """The Rb column from the driver's ~/output (each output's pulse, read back by the driver);
        a pin the sysfs probe reads as off or unreadable keeps that row, so a disabled channel still shows."""
        sysfs = self.state.health.pwm_sysfs
        self.state.health.pwm_readback = [row if index < len(sysfs) and not str(row := sysfs[index]).startswith('ON')
                             else f'ON {pulse} us' for index, pulse in enumerate(pulses)]

    def feed_map(self):
        """The recorded costmap follows the Recorder mode and the session owner (map.md 4)."""
        feeds = getattr(self.ros, 'map', None)
        if feeds is not None:
            nav = any(n.rsplit('/', 1)[-1] == 'planner_server' for n in self.ros.nodes())
            feeds.want_record(not self.watching, self.recorder.mode != 'off', nav)

    @property
    def facts(self):
        """The health facts by name, among them the perc.<source>.* ones of the perception probe."""
        return self.health.facts.read()[0]

    def power_facts(self):
        """The latest ``power.*`` facts (ros.health feeds them), or None without a voltage source."""
        return getattr(self.ros, 'power_facts', None)

    @property
    def engine(self):
        """The health engine behind the advisory rows (gates, snapshots), or None."""
        return getattr(self.health, 'engine', None)

    def health_tick(self, facts, probes, session=None):
        """Evaluate health and journal its transitions, so trial windows can be judged."""
        report = self.engine.tick(facts, probes, session)
        self.journal.record(report)
        return report

    def health_snapshot(self, label, procedure):
        """Journal and emit the shown health; 'end' is taken before teardown."""
        if self.engine is None or self.engine.last is None:
            return
        entry = self.journal.snapshot(label, self.engine.last, self.engine.clock())
        self.ros.emit('health_snapshot', procedure=procedure.key, **entry)
