from types import SimpleNamespace as NS

from mhseals_testing.health.probes import Facts
from mhseals_testing.perception.probe import Probe
from mhseals_testing.ros import perception as feed


def message(stamp, labels=(), frame='camera'):
    sec = int(stamp)
    header = NS(stamp=NS(sec=sec, nanosec=int((stamp - sec) * 1e9)), frame_id=frame)
    detections = [NS(results=[NS(hypothesis=NS(class_id=label, score=0.8))]) for label in labels]
    detections.append(NS(results=[]))       # a detection without a hypothesis is skipped
    return NS(header=header, detections=detections)


def test_source_name_follows_the_clock():
    assert feed.source_name(NS(use_sim_time=True)) == 'sim'
    assert feed.source_name(NS(use_sim_time=False)) == 'zed_native'


def test_detections_become_perc_facts_the_popup_reads():
    from mhseals_testing.ui import perception
    now = [100.0]
    probe, facts = Probe('sim', lambda: now[0]), Facts()
    for _ in range(20):
        now[0] += 0.1
        feed.observe_message(probe, message(now[0] - 0.05, ['buoy', 'dock']))
    facts.publish('perception', probe.facts())
    store = facts.read()[0]
    assert store['perc.sim.hz'].value > 5
    assert store['perc.sim.latency'].value is not None
    assert perception.sources(store) == ['sim']
    assert perception.modal(store)['title'].endswith('sim')
    assert set(store['perc.sim.classes'].data) == {'buoy', 'dock'}


def test_empty_array_counts_as_a_frame():
    now = [10.0]
    probe = Probe('sim', lambda: now[0])
    now[0] += 0.1
    feed.observe_message(probe, message(now[0] - 0.01))
    assert probe.facts()['perc.sim.empty_ratio'].value == 1.0


def test_app_facts_are_the_health_store():
    from mhseals_testing.ui.app import App
    store = Facts()
    store.publish('perception', Probe('sim', lambda: 5.0).facts())   # empty before a frame
    probe = Probe('sim', lambda: 5.0)
    feed.observe_message(probe, message(4.9))
    store.publish('perception', probe.facts())
    host = NS(health=NS(facts=store))
    assert 'perc.sim.hz' in App.facts.fget(host)
