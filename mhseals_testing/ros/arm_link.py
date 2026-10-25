"""Remote arming: the thruster driver and command_mux on the boat computer, kept alive by the TUI.

An operator TUI on a computer without the ``thrusters`` capability (tranquility) arms the one host
that has it (devices.yaml) instead of refusing. There the driver children run under this module
as the unit ``mhseals-arming`` (harness/unit.py, ``--driver``), so they live only while the TUI
is heard over DDS, never on a lease renewed over ssh:

* the TUI publishes ``{token, armed: true}`` on ``/arm_link/beat`` every ``BEAT_S``;
* the boat side starts the children only after the first beat with its token, and stops them
  (SIGINT to each child and its descendants: ``ros2 run`` does not forward it, then TERM, KILL)
  once no beat came for ``LINK_S``, on ``armed: false`` with its token, when any child exits, or
  on a signal; it then exits, and the keeper stops whatever is left of the group;
* the keeper's own lease (``LEASE_S``, renewed by the boat side's loop) stops everything when
  this process wedges;
* ``/arm_link/state`` (``STATE_S``) says which children run and why it stopped, so the TUI sees a
  dead driver within ``STATE_STALE_S``.

Bounds on link loss: manual commands are neutral after the mux's ``cmd_timeout`` (0.5 s), as with
a local driver; the driver and mux stop after ``LINK_S`` (1 s), which a local driver never does
(its keeper follows the TUI process, not the link). Disarm and e-stop act over DDS first (the
stop message, ``/thruster_pwm_node/estop``), then over ssh (``unit stop``).

``python3 -m mhseals_testing.ros.arm_link run --token T --config-b64 B [--unit U] -- name argv...
[--next name argv...]`` (``@CONFIG@`` in an argv is the shipped hardware config's path);
``readback --config-b64 B -- argv...`` runs the driver alone for the pin readback, then stops it.
"""

import argparse
import base64
import concurrent.futures
import ctypes
import json
import os
from pathlib import Path
import secrets
import shlex
import signal
import subprocess
import sys
import threading
import time

BEAT_TOPIC = '/arm_link/beat'
STATE_TOPIC = '/arm_link/state'
ESTOP_SERVICE = '/thruster_pwm_node/estop'
DRIVER_STATUS = '/thruster_pwm_node/status'
BEAT_S = 0.1            # the TUI beats at 10 Hz
LINK_S = 1.0            # without a beat this long the boat stops the driver and the mux
STATE_S = 0.2
STATE_STALE_S = 2.0     # the TUI faults on a boat state older than this
FIRST_BEAT_S = 15.0     # the boat waits this long for the first beat (discovery over the radio)
STOP_REPEAT_S = 2.0     # the TUI repeats armed: false this long after a disarm
LEASE_S = 3.0           # keeper backstop for a wedged boat side
START_TIMEOUT_S = 30.0  # TUI: ssh, discovery and the driver's start
READBACK_S = 40.0
UNIT = 'mhseals-arming'
CONFIG = '@CONFIG@'
NEXT = '--next'
MODULE = ('python3', '-m', 'mhseals_testing.ros.arm_link')
DRIVERS = {'thruster_pwm_node', 'command_mux'}
WRAPPERS = {'mhseals_testing.ros.arm_link', 'mhseals_testing.harness.keeper',
            'mhseals_testing.harness.unit'}
PR_SET_PDEATHSIG = 1


# --- boat side --------------------------------------------------------------------------------

class Watch:
    """Why the children must stop now, or '' (ROS-free; fed the beat messages)."""

    def __init__(self, token, link_s=LINK_S, first_beat_s=FIRST_BEAT_S, now=None):
        self.token, self.link_s, self.first_beat_s = token, link_s, first_beat_s
        self.started = time.monotonic() if now is None else now
        self.beat_at, self.stopped = None, ''

    def message(self, data, now):
        try:
            found = json.loads(data)
        except ValueError:
            return
        if not isinstance(found, dict) or found.get('token') != self.token:
            return              # another TUI's beat never keeps this driver alive
        if found.get('armed') is True:
            self.beat_at = now
        else:
            self.stopped = 'disarmed by the operator'

    def heard(self):
        return self.beat_at is not None

    def reason(self, now):
        if self.stopped:
            return self.stopped
        if self.beat_at is None:
            return (f'no operator beat within {self.first_beat_s:.0f} s'
                    if now - self.started > self.first_beat_s else '')
        if now - self.beat_at > self.link_s:
            return f'operator link lost (no beat for {self.link_s:g} s)'
        return ''


def split_children(words):
    """``[(name, argv)]`` from ``name argv... --next name argv...``."""
    groups, current = [], []
    for word in words:
        if word == NEXT:
            groups.append(current)
            current = []
        else:
            current.append(word)
    groups.append(current)
    if not groups or any(len(group) < 2 for group in groups):
        raise ValueError('each child needs a name and a command')
    return [(group[0], group[1:]) for group in groups]


def join_children(children):
    words = []
    for name, argv in children:
        words += ([NEXT] if words else []) + [name, *argv]
    return words


def rivals(proc=Path('/proc')):
    """Pids of a thruster driver or mux already on this computer (any TUI's, any container
    sharing the pid namespace), found by command line, not by the DDS graph a slow link hides."""
    found = []
    for entry in proc.iterdir():
        if not entry.name.isdigit() or int(entry.name) == os.getpid():
            continue
        try:
            words = [word.decode(errors='replace')
                     for word in (entry / 'cmdline').read_bytes().split(b'\0') if word]
        except OSError:
            continue
        if WRAPPERS & set(words):
            continue      # a keeper, unit or arm_link naming the driver in its argv
        if DRIVERS & {Path(word).name for word in words}:
            found.append(int(entry.name))
    return sorted(found)


def descendants(pid, proc=Path('/proc')):
    """``pid``'s children, grandchildren, ... (from /proc stat)."""
    parents = {}
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / 'stat').read_text()
        except OSError:
            continue
        parents.setdefault(int(stat.rsplit(')', 1)[1].split()[1]), []).append(int(entry.name))
    found, todo = [], [pid]
    while todo:
        for child in parents.get(todo.pop(), []):
            found.append(child)
            todo.append(child)
    return found


def _prepare_child():
    """Stay in the keeper's group (its stop reaches us all); SIGINT when this process dies."""
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    signal.signal(signal.SIGTERM, signal.SIG_DFL)
    if sys.platform.startswith('linux'):
        ctypes.CDLL(None, use_errno=True).prctl(PR_SET_PDEATHSIG, signal.SIGINT)


class Child:
    """One child, its output copied to ours with its name, its last line kept."""

    def __init__(self, name, argv):
        self.name, self.last = name, ''
        self.process = subprocess.Popen(argv, preexec_fn=_prepare_child, stdin=subprocess.DEVNULL,
                                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        threading.Thread(target=self._copy, daemon=True).start()

    def _copy(self):
        for raw in self.process.stdout:
            line = raw.decode(errors='replace').rstrip()
            if line:
                self.last = line[-200:]
            sys.stdout.write(f'[{self.name}] {line}\n')
            sys.stdout.flush()

    def running(self):
        return self.process.poll() is None


def stop_children(children, waits=(5.0, 2.0, 2.0)):
    """INT (the driver neutralizes and cuts ESC power as it exits), TERM, KILL: each child and
    every descendant, since ``ros2 run`` does not pass a signal on to its node."""
    for signum, wait in zip((signal.SIGINT, signal.SIGTERM, signal.SIGKILL), waits):
        targets = [pid for child in children if child.running()
                   for pid in (child.process.pid, *descendants(child.process.pid))]
        for child in children:      # a wrapper may be gone while its node is not
            if not child.running():
                targets += descendants(child.process.pid)
        if not targets:
            break
        for pid in targets:
            try:
                os.kill(pid, signum)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + wait
        while time.monotonic() < deadline and any(_alive(pid) for pid in targets):
            for child in children:
                child.process.poll()
            time.sleep(0.05)
    for child in children:
        child.process.poll()


def _alive(pid):
    try:
        with open(f'/proc/{pid}/stat') as handle:
            return handle.read().rsplit(')', 1)[1].split()[0] != 'Z'
    except OSError:
        return False


def write_config(encoded, name):
    """Write the shipped hardware config under the data root; its path."""
    from mhseals_testing import data
    target = data.path('units', UNIT, f'{time.strftime("%Y%m%d_%H%M%S")}_{name}.hardware.yaml')
    target.parent.mkdir(parents=True, exist_ok=True)
    data.atomic_write(target, base64.b64decode(encoded, validate=True).decode())
    return target


def with_config(argv, path):
    return [word.replace(CONFIG, str(path)) for word in argv]


def _touch(lease):
    if lease is not None:
        try:
            os.utime(lease)
        except OSError:
            pass


def _ros(name):
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)   # ours stop the children
    node = rclpy.create_node(name)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spinning = [True]

    def spin():
        while spinning[0] and rclpy.ok():
            executor.spin_once(timeout_sec=0.05)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()

    def close():
        spinning[0] = False
        thread.join(timeout=1)
        executor.shutdown(timeout_sec=1)
        node.destroy_node()
        rclpy.try_shutdown()
    return node, close


def _qos():
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def run(token, encoded, children, lease=None, link_s=LINK_S, first_beat_s=FIRST_BEAT_S):
    """Hold the children while the operator's beats arrive; the exit code is 0 on a disarm."""
    _touch(lease)
    stopping = []
    for signum in (signal.SIGTERM, signal.SIGHUP, signal.SIGINT):
        signal.signal(signum, lambda number, _: stopping.append(f'signal {number}'))
    config = write_config(encoded, 'arm')
    children = [(name, with_config(argv, config)) for name, argv in children]
    found = rivals()
    if found:
        print(json.dumps({'error': f'a thruster driver or mux already runs here (pids {found})'}),
              flush=True)
        return 2
    from std_msgs.msg import String
    node, close = _ros('arm_link')
    watch = Watch(token, link_s, first_beat_s)
    node.create_subscription(String, BEAT_TOPIC,
                             lambda message: watch.message(message.data, time.monotonic()), _qos())
    state = node.create_publisher(String, STATE_TOPIC, _qos())
    running, reason, published = [], '', 0.0

    def publish(phase):
        state.publish(String(data=json.dumps({
            'token': token, 'phase': phase, 'reason': reason, 'config': str(config),
            'children': {child.name: child.running() for child in running},
            'last': {child.name: child.last for child in running}})))

    try:
        while True:
            now = time.monotonic()
            reason = stopping[0] if stopping else watch.reason(now)
            if not reason and watch.heard() and not running:
                print(f'arm_link: operator heard; starting {", ".join(n for n, _ in children)}',
                      flush=True)
                running = [Child(name, argv) for name, argv in children]
            exited = [child for child in running if not child.running()]
            if exited and not reason:
                reason = f'{exited[0].name} exited ({exited[0].process.returncode})'
            _touch(lease)
            if reason:
                break
            if now - published >= STATE_S:
                publish('running' if running else 'waiting')
                published = now
            time.sleep(0.05)
        print(f'arm_link: stopping: {reason}', flush=True)
        publish('stopping')
        stop_children(running)
        for _ in range(3):
            publish('stopped')
            time.sleep(0.05)
    finally:
        stop_children(running)
        close()
    return 0 if watch.stopped and not stopping else 3


def readback(encoded, argv, timeout=None):
    """Run only the driver from the shipped config, read its PWM outputs back from sysfs, stop it."""
    from mhseals_hardware.configuration import load_config
    from mhseals_testing.ui.popups import pins
    config = write_config(encoded, 'readback')
    found = rivals()
    if found:
        return {'error': f'a thruster driver or mux already runs here (pids {found})'}
    child = Child('hardware', with_config(argv, config))
    try:
        return {'confirmed': pins.readback(load_config(config))}
    finally:
        stop_children([child])      # outputs disabled, ESC power off


def main(argv=None):
    parser = argparse.ArgumentParser(prog='python3 -m mhseals_testing.ros.arm_link')
    verbs = parser.add_subparsers(dest='verb', required=True)
    hold = verbs.add_parser('run')
    hold.add_argument('--token', required=True)
    hold.add_argument('--unit')
    hold.add_argument('--link-s', type=float, default=LINK_S)
    hold.add_argument('--first-beat-s', type=float, default=FIRST_BEAT_S)
    for verb in (hold, verbs.add_parser('readback')):
        verb.add_argument('--config-b64', required=True)
    argv = sys.argv[1:] if argv is None else list(argv)
    at = argv.index('--') if '--' in argv else len(argv)
    args, words = parser.parse_args(argv[:at]), argv[at + 1:]
    if not words:
        parser.error('the children follow --')
    if args.verb == 'readback':
        result = readback(args.config_b64, words)
        print(json.dumps(result), flush=True)
        return 0 if 'error' not in result else 2
    if not 0 < args.link_s <= 5.0:
        parser.error('--link-s must be over 0 and at most 5 s')
    lease = None
    if args.unit:
        from mhseals_testing.harness import unit
        lease = unit.units_dir() / f'{args.unit}.lease'
    return run(args.token, args.config_b64, split_children(words), lease, args.link_s,
               args.first_beat_s)


# --- TUI side ---------------------------------------------------------------------------------

class RosLink:
    """The TUI's end on its own node: the beat timer (executor thread), the boat's state, e-stop."""

    def __init__(self, node, clock=time.monotonic):
        from std_msgs.msg import String
        self.String, self.node, self.clock = String, node, clock
        self.lock = threading.Lock()
        self.token, self.armed, self.stop_until, self.latest = None, False, 0.0, (None, None)
        self.publisher = node.create_publisher(String, BEAT_TOPIC, _qos())
        node.create_subscription(String, STATE_TOPIC, self._state, _qos())
        self.timer = node.create_timer(BEAT_S, self._beat)
        from std_srvs.srv import Trigger
        self.Trigger = Trigger    # the client exists from the start, so the service is matched long before an e-stop
        self.client = node.create_client(Trigger, ESTOP_SERVICE)

    def _message(self):
        return self.String(data=json.dumps({'token': self.token, 'armed': self.armed}))

    def _beat(self):
        with self.lock:
            if self.token is not None and (self.armed or self.clock() < self.stop_until):
                self.publisher.publish(self._message())

    def _state(self, message):
        try:
            found = json.loads(message.data)
        except ValueError:
            return
        if isinstance(found, dict):
            with self.lock:
                self.latest = (found, self.clock())

    def set(self, token, armed):
        """Beat for ``token``; ``armed`` False sends the stop now, and again for STOP_REPEAT_S."""
        with self.lock:
            self.token, self.armed = token, armed
            if not armed:
                self.stop_until = self.clock() + STOP_REPEAT_S
                for _ in range(3):
                    self.publisher.publish(self._message())

    def state(self):
        with self.lock:
            return self.latest

    def estop(self):
        """Ask the driver to cut and latch ESC power now; never waits (the answer may not come)."""
        self.client.call_async(self.Trigger.Request())


class RemoteDriver:
    """Arming's children on another computer, as the unit ``mhseals-arming`` behind ``link``.

    ssh calls run one at a time off the UI thread, so a stop always precedes a later start.
    """

    timeout = START_TIMEOUT_S

    def __init__(self, host, link, jump=None, by=None, call=None, clock=time.monotonic):
        from mhseals_testing.harness import remote
        self.host, self.link, self.jump, self.by = host, link, jump, dict(by or {})
        self.call = call or remote.call
        self.clock = clock
        self.token, self.error = None, ''
        self.worker = concurrent.futures.ThreadPoolExecutor(1, thread_name_prefix='arm_link')

    def describe(self):
        return f'{self.host.name} (stops {LINK_S:g} s after the link drops)'

    def _job(self, function, *args):
        return self.worker.submit(function, *args)

    def start(self, children, config_text):
        """Beat first, then ask the boat to start: it waits for the beat before any child."""
        from mhseals_testing.harness.remote import unit_command
        self.token, self.error = secrets.token_hex(6), ''
        self.link.set(self.token, True)
        argv = [*MODULE, 'run', '--token', self.token, '--unit', UNIT,
                '--config-b64', base64.b64encode(config_text.encode()).decode(),
                '--', *join_children(children)]
        self._job(self._start, self.token,
                  unit_command('start', UNIT, argv, self.by, lease_s=LEASE_S, driver=True))

    def _start(self, token, command):
        from mhseals_testing.harness.remote import CONNECT_S
        from mhseals_testing.harness.unit import START_WAIT
        results, error = self.call(self.host, command, self.jump, CONNECT_S + START_WAIT + 10)
        found = results[0] if results else {}
        if token != self.token:
            return
        if error or 'error' in found:
            self.error = f'{self.host.name}: {error or found["error"]}'
        elif found.get('adopted'):
            by = (found.get('by') or {}).get('host') or 'another TUI'
            self.error = (f'{UNIT} already runs on {self.host.name} (armed from {by}); '
                          'disarm it there, or wait a few seconds after a disarm')
        if self.error:
            self.link.set(token, False)

    def _state(self):
        state, at = self.link.state()
        if state is None or self.token is None or state.get('token') != self.token:
            return None, None
        return state, at

    def confirmed(self):
        """The boat reports every child running for this arming."""
        state, at = self._state()
        return (state is not None and state.get('phase') == 'running'
                and self.clock() - at <= STATE_STALE_S)

    def running(self, name):
        """False once the boat says ``name`` stopped, or its state goes stale; True while it starts."""
        if self.error:
            return False
        state, at = self._state()
        if state is None:
            return True             # not heard yet: Arming's start timeout decides
        if self.clock() - at > STATE_STALE_S:
            return False
        if state.get('phase') == 'waiting':
            return True
        return state.get('phase') == 'running' and bool((state.get('children') or {}).get(name))

    def last_line(self, name):
        if self.error:
            return self.error
        state, at = self._state()
        if state is None:
            return ''
        if at is not None and self.clock() - at > STATE_STALE_S:
            return f'no state from {self.host.name} for {self.clock() - at:.1f} s (link?)'
        reason, last = state.get('reason') or '', (state.get('last') or {}).get(name) or ''
        return f'{reason} ({name}: {last})' if reason and last else reason or last

    def stop(self):
        """DDS first (the stop message, the driver's e-stop), then ``unit stop`` over ssh."""
        if self.token is None:
            return
        self.link.set(self.token, False)
        try:
            self.link.estop()
        finally:
            self._job(self._stop)

    def _stop(self):
        from mhseals_testing.harness.remote import CONNECT_S, unit_command
        from mhseals_testing.harness.unit import STOP_WAIT
        return self.call(self.host, unit_command('stop', UNIT), self.jump, CONNECT_S + STOP_WAIT + 10)

    def readback(self, config_text, argv):
        """Run the pin readback on the boat (its sysfs); True when the new channels showed enabled."""
        command = shlex.join([*MODULE, 'readback', '--config-b64',
                              base64.b64encode(config_text.encode()).decode(), '--', *argv])
        results, error = self._job(self.call, self.host, command, self.jump, READBACK_S).result()
        found = results[-1] if results else {}
        if error or 'error' in found:
            raise RuntimeError(f'{self.host.name}: {error or found["error"]}')
        return bool(found.get('confirmed'))


def target(profile, registry=None):
    """``(host, jump)`` whose thrusters this TUI arms over the link, or None (arm here, or refuse).

    Only an ``odroid``-output profile on a computer without ``thrusters`` arms remotely, and only
    when exactly one other ssh-reachable host has the capability.
    """
    if profile.outputs != 'odroid':
        return None
    from mhseals_testing import devices
    try:
        me = profile.placed[0]
        registry = registry or devices.load()
    except devices.DeviceError:
        return None
    if 'thrusters' in me.capabilities:
        return None
    found = [host for host in registry.hosts.values()
             if 'thrusters' in host.capabilities and host.ssh and host.name not in (me.name, devices.FALLBACK)]
    if len(found) != 1:
        return None
    return found[0], registry.hosts.get(found[0].via) if found[0].via else None


if __name__ == '__main__':
    sys.exit(main())
