"""The agent channel file and config/agent.yaml (spec agent-control sections 2 and 4.1), no ROS."""

import json
import os

import pytest

from mhseals_testing.harness import agent_channel as ch
from mhseals_testing.harness import agent_config


def test_lines_carry_the_common_fields_and_land_under_the_data_root(data_root):
    req = ch.request('run', 'claude-3f2a', {'key': 'square'}, claim='c4', dry=True, request_s=5)
    assert ch.channel_path() == data_root / 'agent' / 'channel.jsonl'
    [stored] = [json.loads(x) for x in ch.channel_path().read_text().splitlines()]
    assert stored == req and list(stored)[0] == 'schema' and stored['schema'] == ch.SCHEMA
    assert {'t_ns', 'host', 'pid', 'who', 'kind'} <= set(stored) and stored['kind'] == 'req'
    assert len(stored['id']) == 32 and stored['dry'] is True and stored['claim'] == 'c4'
    assert 4.9e9 < stored['deadline_ns'] - stored['t_ns'] < 5.1e9
    assert ch.invalid(stored) is None


def test_tail_reads_each_kind_once_and_keeps_its_position():
    req = ch.request('claim', 'a', {'reason': 'tune'})
    ch.renew('a', 'c1', req['id'])
    ch.note('a', 'pre', 'ssh blastoise ros2 topic hz /odom TOKEN=abc', tool_id='t1')
    ch.reply(req['id'], 'tui', state='started')
    tail = ch.Tail()
    records, errors = tail.read()
    assert [r['kind'] for r in records] == ['req', 'renew', 'note', 'reply'] and not errors
    assert records[2]['cmd'] == 'ssh blastoise ros2 topic hz /odom TOKEN=…'
    assert tail.read() == ([], [])
    again = ch.Tail(**tail.position())
    assert again.read() == ([], [])
    ch.note('a', 'text', 'hello')
    assert [r['cmd'] for r in again.read()[0]] == ['hello']


def test_bounded_reads_skip_malformed_lines_and_wait_for_torn_ones():
    path = ch.channel_path()
    for i in range(100):
        ch.note('a', 'text', str(i))
    with open(path, 'a') as handle:
        handle.write('not json\n{"schema": "mhseals.agent/1", "kind": "req", "who": "a", "t_ns": 1}\n')
    tail = ch.Tail()
    first, _ = tail.read(max_lines=64)
    assert len(first) == 64
    rest, errors = tail.read(max_lines=64)
    assert len(rest) == 36 and errors == ['malformed JSON line', 'req without id, op, deadline_ns']
    with open(path, 'a') as handle:
        handle.write('{"half": ')
    assert tail.read() == ([], [])
    ch.note('a', 'text', 'after')     # append_jsonl fences the torn line off
    records, errors = tail.read()
    assert [r['cmd'] for r in records] == ['after'] and errors == ['malformed JSON line']


def test_a_raw_carriage_return_never_stops_the_tail():
    ch.channel_path().parent.mkdir(parents=True)
    with open(ch.channel_path(), 'ab') as handle:
        handle.write(b'{"bad\rline": 1}\n')
    ch.note('a', 'text', 'after')
    tail = ch.Tail()
    records, errors = tail.read()
    assert [r['cmd'] for r in records] == ['after'] and errors == ['malformed JSON line']
    ch.note('a', 'text', 'later')
    assert [r['cmd'] for r in tail.read()[0]] == ['later']


@pytest.mark.parametrize('text, secret', [
    ('curl --password hunter2 x', 'hunter2'), ('curl --token=abc123 x', 'abc123'),
    ('sshpass -p hunter2 ssh blastoise', 'hunter2'), ('sshpass -phunter2 ssh blastoise', 'hunter2'),
    ('curl -H "Authorization: Bearer abc.def" x', 'abc.def'), ('wget --header="Bearer s3cr3t"', 's3cr3t'),
    ('PASSWORD="two words" ros2 run x', 'two words'), ("token='quoted value' x", 'quoted value')])
def test_redact_hides_flag_header_and_quoted_secrets(text, secret):
    hidden = ch.redact(text)
    assert secret not in hidden and '…' in hidden


def test_free_text_args_are_redacted_on_the_channel():
    ch.request('claim', 'a', {'reason': 'tune token=tokval'})
    ch.request('ask', 'a', {'question': 'use sshpass -p hunter2?', 'options': ['yes']})
    ch.request('mark', 'a', {'note': 'Authorization: Bearer sekrit'})
    text = ch.channel_path().read_text()
    assert not any(s in text for s in ('tokval', 'hunter2', 'sekrit')) and '"options":["yes"]' in text


def test_tail_starts_over_on_rotation_or_truncation():
    ch.note('a', 'text', 'old')
    tail = ch.Tail()
    tail.read()
    assert ch.rotate(active=False, limit=10) is not None
    ch.note('a', 'text', 'new')
    assert [r['cmd'] for r in tail.read()[0]] == ['new']
    ch.channel_path().write_text('')
    ch.note('a', 'text', 'x')
    assert [r['cmd'] for r in tail.read()[0]] == ['x']


def test_rotation_only_when_big_and_no_claim_is_active():
    ch.note('a', 'text', 'x' * 100)
    assert ch.rotate(active=True, limit=10) is None
    assert ch.rotate(active=False, limit=10_000) is None
    moved = ch.rotate(active=False, limit=10, now=0)
    assert moved.name.startswith('channel.19') and moved.suffix == '.jsonl'
    assert not ch.channel_path().exists()
    ch.note('a', 'text', 'x' * 100)
    again = ch.rotate(active=False, limit=10, now=0)     # the same second: a new name, nothing lost
    assert again != moved and moved.exists() and again.exists()


def test_replied_finds_requests_already_answered_after_a_position():
    old = ch.request('run', 'a')
    position = ch.Tail().seek_end()
    pending = ch.request('run', 'a')
    answered = ch.request('goal', 'a')
    ch.reply(answered['id'], 'tui', ok=False, error='expired')
    ch.reply(old['id'], 'tui')
    assert ch.replied(position) == {answered['id'], old['id']}
    assert pending['id'] not in ch.replied(position)


def test_seek_end_skips_earlier_lines():
    ch.note('a', 'text', 'before')
    tail = ch.Tail()
    tail.seek_end()
    ch.note('a', 'text', 'after')
    assert [r['cmd'] for r in tail.read()[0]] == ['after']


def test_an_oversized_reply_keeps_its_short_result_keys():
    rec = ch.reply('r1', 'tui', result={'run_file': '/x/run.json', 'results': ['y' * 300] * 20})
    assert rec['result'] == {'run_file': '/x/run.json', 'truncated': True}
    with pytest.raises(ValueError):
        ch.reply('r1', 'tui', result=['y' * 5000])


def test_shipped_agent_config_is_valid():
    config = agent_config.load()
    assert config['lease'] == {'idle_s': 120, 'motion_s': 10, 'max_claim_s': 3600}
    assert 'manual' not in config['run'] and 'square' in config['run']
    assert config['unclaimed_driver'] == 'warn' and config['hook']['max_per_10s'] == 6


@pytest.mark.parametrize('edit, word', [
    (lambda c: c.update(schema='x'), 'schema'),
    (lambda c: c['lease'].update(idle_s=0), 'lease.idle_s'),
    (lambda c: c['lease'].update(motion_s=500), 'motion_s <= idle_s'),
    (lambda c: c['lease'].pop('max_claim_s'), 'lease needs'),
    (lambda c: c.update(request_s='5'), 'request_s'),
    (lambda c: c.update(run='square'), 'run'),
    (lambda c: c.update(unclaimed_driver='stop'), 'unclaimed_driver'),
    (lambda c: c['hook']['deny'].append('('), 'hook.deny'),
    (lambda c: c.update(extra=1), 'unknown key extra'),
])
def test_bad_agent_config_names_the_key(edit, word):
    config = json.loads(json.dumps(agent_config.load()))
    edit(config)
    with pytest.raises(ValueError, match=word):
        agent_config.validate(config)


def test_watch_hot_reloads_and_keeps_the_last_good_config(tmp_path):
    path = tmp_path / 'agent.yaml'
    text = agent_config.config_path().read_text()
    path.write_text(text)
    watch = agent_config.Watch(path)
    assert watch.get()['lease']['idle_s'] == 120 and watch.error is None
    path.write_text(text.replace('idle_s: 120', 'idle_s: 60'))
    os.utime(path, ns=(1, 1))
    assert watch.get()['lease']['idle_s'] == 60
    path.write_text(text.replace('idle_s: 120', 'idle_s: -1'))
    assert watch.get()['lease']['idle_s'] == 60 and 'idle_s' in watch.error
    path.write_text('lease: [')
    os.utime(path, ns=(2, 2))
    assert watch.get()['lease']['idle_s'] == 60 and watch.error
    path.unlink()
    assert watch.get()['lease']['idle_s'] == 60 and watch.error


def test_reply_errors_map_to_the_cli_exit_codes():
    assert ch.exit_code({'ok': True}) == 0
    assert [ch.exit_code({'ok': False, 'error': e}) for e in
            ('busy', 'motion_observe', 'overridden: rc', 'expired', 'no_owner', 'blocked', 'timeout', 'odd')] == \
        [2, 2, 3, 3, 4, 5, 6, 1]
