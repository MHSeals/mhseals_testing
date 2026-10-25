from types import SimpleNamespace as NS

import pytest

from mhseals_testing.health.probes import Facts
from mhseals_testing.perception.probe import Probe
from mhseals_testing.ros import perception as feed


def array(*statuses):
    return NS(status=[NS(name=name, level=0, message='', hardware_id=hw,
                         values=[NS(key=k, value=v) for k, v in values]) for name, hw, values in statuses])


BACKEND = ('perception/sim/backend', 'crane', [('hz', '9.8'), ('latency_ms', '50')])


def test_source_name_follows_the_clock():
    assert feed.source_name(NS(use_sim_time=True)) == 'sim'
    assert feed.source_name(NS(use_sim_time=False)) == 'zed_native'


def test_status_becomes_perc_facts_the_popup_reads():
    from mhseals_testing.ui import perception
    probe, facts = Probe('sim', lambda: 1.0), Facts()
    feed.observe_status(probe, array(BACKEND), {'source_timestamp': 10_005_000_000,
                                                'received_timestamp': 10_000_000_000})
    facts.publish('perception', probe.facts())
    store = facts.read()[0]
    assert store['perc.sim.hz'].value == 9.8
    assert store['perc.sim.clock_offset'].value == pytest.approx(0.005)
    assert perception.sources(store) == ['sim']
    assert perception.modal(store)['title'].endswith('sim')


def test_missing_message_info_gives_no_clock_offset():
    probe = Probe('sim', lambda: 1.0)
    feed.observe_status(probe, array(BACKEND))
    feed.observe_status(probe, array(BACKEND), {'source_timestamp': 0, 'received_timestamp': 1})
    assert 'perc.sim.clock_offset' not in probe.facts()


def test_only_zed_diagnostics_reach_the_inference_fact():
    probe = Probe('zed_native', lambda: 1.0)
    feed.observe_diagnostics(probe, array(
        ('mavros', 'fcu', [('Object detection', 'Processing Time: 0.9 sec')]),
        ('zed', 'Stereolabs camera', [('Object detection', 'Processing Time: 0.05 sec')])))
    assert probe.facts()['perc.zed_native.inference_ms'].value == pytest.approx(50.0)


def test_subscribes_only_the_two_status_topics():
    pytest.importorskip('diagnostic_msgs')
    subs, timers = [], []
    node = NS(create_subscription=lambda kind, topic, callback, qos: subs.append((topic, callback)),
              create_timer=lambda period, callback: timers.append(callback))
    facts = Facts()
    probe = feed.subscribe_detections(NS(node=node), facts, NS(use_sim_time=True))
    assert [t for t, _ in subs] == ['/perception/status', '/diagnostics']
    subs[0][1](array(BACKEND), {'source_timestamp': 2, 'received_timestamp': 1})   # rclpy passes MessageInfo
    timers[0]()
    assert facts.read()[0]['perc.sim.hz'].value == 9.8 and probe.source == 'sim'


def test_app_facts_are_the_health_store():
    from mhseals_testing.ui.app import App
    store = Facts()
    probe = Probe('sim', lambda: 1.0)
    feed.observe_status(probe, array(BACKEND))
    store.publish('perception', probe.facts())
    host = NS(health=NS(facts=store))
    assert 'perc.sim.hz' in App.facts.fget(host)


def test_rclpy_hands_the_status_callback_its_dds_stamps():
    rclpy = pytest.importorskip('rclpy')
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    rclpy.init()
    try:
        node = rclpy.create_node('perception_probe_test')
        facts = Facts()
        probe = feed.subscribe_detections(NS(node=node), facts, NS(use_sim_time=True))
        pub = node.create_publisher(DiagnosticArray, feed.STATUS, 10)
        message = DiagnosticArray(status=[DiagnosticStatus(
            name='perception/sim/backend', values=[KeyValue(key='hz', value='9.8')])])
        for _ in range(50):
            pub.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
            if probe.sources['sim'].offsets:
                break
        assert abs(probe.facts()['perc.sim.clock_offset'].value) < 0.5
    finally:
        rclpy.shutdown()
