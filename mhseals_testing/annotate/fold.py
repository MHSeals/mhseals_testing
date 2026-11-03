"""The session journal folded into review items: marks, segments, language commands and the water conditions.

Every item is ``{id, kind, t0, t1, label, source, status, note, effect, detail}`` in wall seconds. Kinds:
    mark          b (a digit tags it), an agent's ``mark``, a system marker
    bad           N on .. N off: data to ignore
    intervention  a person took over from automation (keys or RC over a procedure, agent, task or goal)
    procedure     <name>_start .. <name>_stop, with its outcome; step: a characterization step inside one
    armed         armed .. disarmed
    hold, link    a mux hold (link: a link-loss hold), from the live feed's hold_on .. hold_off
    fence         the boat left the course area
    fix           one GNSS fix tier, from the live feed's fix edges
    steady        a command held still (neutral: at zero), a calibration candidate
    said          an operator's language instruction, with the pose and scene

Review edits are ``label_edit`` lines (accept, reject, pending, tag, note, outcome, text, split, merge, undo),
applied in order, so the recorded lines are never rewritten. ROS-free and pure.
"""

from mhseals_testing.annotate import config

AUTO = ('agent', 'task', 'procedure', 'nav')          # owners that are automation
HUMAN = ('keyboard', 'rc')
NOT_PROCEDURES = ('session', 'rec', 'pause')
FIX_EFFECT = {'none': 'invalid', '2D': 'invalid', '3D': 'degraded', 'DGPS': 'degraded', 'FLOAT': 'degraded',
              'FIXED': 'none'}
LABELLED = ('procedure', 'step', 'steady', 'neutral', 'armed')   # the windows a consumer fits on
EFFECT_OF = {'bad': 'invalid', 'intervention': 'invalid', 'link': 'invalid', 'fence': 'suspect'}


def seconds(event):
    return event['t_ns'] / 1e9 if isinstance(event.get('t_ns'), (int, float)) else None


def item(kind, t0, t1=None, label='', source='auto', **detail):
    return {'id': f'{kind}:{round(t0 * 1000)}', 'kind': kind, 't0': t0, 't1': t0 if t1 is None else t1,
            'label': label, 'source': source, 'status': 'pending', 'note': '', 'effect': EFFECT_OF.get(kind, 'none'),
            'detail': detail}


class _Folder:
    def __init__(self):
        self.items, self.open, self.conditions, self.edits, self.end = {}, {}, [], [], 0.0
        self.owner = None

    def add(self, entry):
        while entry['id'] in self.items:          # two edges in one millisecond
            entry['id'] += "'"
        self.items[entry['id']] = entry
        return entry

    def start(self, key, entry):
        self.close(key, entry['t0'])
        self.open[key] = self.add(entry)

    def close(self, key, t, **detail):
        entry = self.open.pop(key, None)
        if entry is not None:
            entry['t1'] = t
            entry['detail'].update(detail)
        return entry

    def feed(self, event):
        t, phase = seconds(event), event.get('phase', '')
        if t is None:
            return
        self.end = max(self.end, t)
        handler = getattr(self, f'on_{phase}', None)
        if handler is not None:
            handler(event, t)
        elif phase.endswith('_start') and phase[:-6] not in NOT_PROCEDURES:
            name = phase[:-6]
            self.close('step', t)
            self.start(('procedure', name), item('procedure', t, label=name, key=event.get('key', ''),
                                                 run=event.get('run_id')))
        elif phase.endswith('_stop') and ('procedure', phase[:-5]) in self.open:
            self.close('step', t)
            entry = self.close(('procedure', phase[:-5]), t, outcome=str(event.get('outcome', '')), by=event.get('by'))
            if event.get('by') in ('operator', 'keyboard') and entry['detail']['outcome'] != 'complete':
                self.add(item('intervention', t, label=f"{entry['label']} aborted", by=event.get('by')))

    # operator and agent marks
    def on_marker(self, e, t):
        tag = e.get('tag') or ''
        self.add(dict(item('mark', t, label=tag, source=e.get('kind', 'user'), who=e.get('who'), why=e.get('why')),
                      id=f"mark:{e.get('id')}", note=e.get('note', ''), effect=config.tag(tag)['effect'] if tag else 'none'))

    def on_marker_note(self, e, t):
        entry = self.items.get(f"mark:{e.get('id')}")
        if entry is not None:
            entry['note'] = e.get('note', '')

    def on_marker_tag(self, e, t):
        entry = self.items.get(f"mark:{e.get('id')}")
        if entry is not None:
            entry['label'], entry['effect'] = e.get('tag', ''), config.tag(e.get('tag', ''))['effect']

    def on_bad_data(self, e, t):
        if e.get('on'):
            self.start('bad', item('bad', t, label='bad data', source='operator', who=e.get('who')))
        else:
            self.close('bad', t)

    def on_conditions_set(self, e, t):
        self.conditions.append((t, {k: int(e.get(k) or 0) for k in config.load()['scales']}))

    def on_said(self, e, t):
        self.add(dict(item('said', t, label=e.get('text', ''), source=e.get('source', 'operator'),
                           pose=e.get('pose'), scene=e.get('scene'), outcome=''), id=f"said:{e.get('id')}"))

    def on_said_text(self, e, t):
        entry = self.items.get(f"said:{e.get('id')}")
        if entry is not None:
            entry['label'] = e.get('text', '')

    # automatic edges
    def on_step(self, e, t):
        self.start('step', item('step', t, label=e.get('label', ''), done=e.get('done'), total=e.get('total')))

    def on_armed(self, e, t):
        self.start('armed', item('armed', t, label='armed', by=e.get('by', 'operator'), who=e.get('who')))

    def on_disarmed(self, e, t):
        self.close('armed', t, reason=e.get('reason'), ended_by=e.get('by'))

    def on_hold_on(self, e, t):
        cause = str(e.get('cause', ''))
        self.start(('hold', cause), item('link' if 'link' in cause else 'hold', t, label=cause, by=e.get('by')))

    def on_hold_off(self, e, t):
        self.close(('hold', str(e.get('cause', ''))), t)

    def on_watchdog_hold(self, e, t):
        self.add(item('hold', t, label=str(e.get('reason', 'watchdog'))))

    def on_fence_outside(self, e, t):
        self.add(item('fence', t, label='outside the course area', on_exit=e.get('on_exit')))

    def on_gnss_fix(self, e, t):
        self.start('fix', item('fix', t, label=str(e.get('fix', 'none')), sats=e.get('sats')))

    def on_owner(self, e, t):
        new, old = e.get('owner'), e.get('prev')
        if new in HUMAN and old in AUTO:
            self.start('intervention', item('intervention', t, label=f'{new} over {old}', taken_from=old, by=new))
        elif new not in HUMAN:
            self.close('intervention', t)

    def on_takeover(self, e, t):
        self.add(item('intervention', t, label='session taken over', by=e.get('by')))

    def on_override(self, e, t):
        self.add(item('intervention', t, label=f"override: {e.get('label') or e.get('by')}", by=e.get('by')))

    def on_claim_revoke(self, e, t):
        self.add(item('intervention', t, label='agent revoked', by=e.get('by')))

    def on_steady(self, e, t):
        t0 = e.get('t0_ns', e['t_ns']) / 1e9
        kind = 'neutral' if not any(e.get('command') or ()) else 'steady'
        self.add(item(kind, t0, t, label=kind, command=e.get('command'), velocity=e.get('velocity'),
                      spread=e.get('spread'), fix=e.get('fix')))

    def on_label_edit(self, e, t):
        self.edits.append(e)


def fold(events):
    """``{t0, t1, items, conditions}`` of a journal (records in any order). Open spans end at the last line."""
    folder = _Folder()
    for event in sorted(events, key=lambda e: (e.get('t_ns') or 0, e.get('seq') or 0)):
        folder.feed(event)
    for key in list(folder.open):
        folder.close(key, folder.end)
    items = apply_edits(folder.items, folder.edits)
    times = [i['t0'] for i in items.values()]
    return {'t0': min(times, default=0.0), 't1': max([folder.end] + [i['t1'] for i in items.values()]),
            'items': sorted(items.values(), key=lambda i: (i['t0'], i['id'])), 'conditions': folder.conditions,
            'edits': len(folder.edits)}


def apply_edits(items, edits):
    """The items after review edits in journal order; ``undo`` drops the latest edit still standing."""
    kept = []
    for edit in edits:
        if edit.get('action') == 'undo':
            if kept:
                kept.pop()
        else:
            kept.append(edit)
    items = {k: dict(v, detail=dict(v['detail'])) for k, v in items.items()}
    for edit in kept:
        entry, action, value = items.get(edit.get('target')), edit.get('action'), edit.get('value')
        if entry is None:
            continue
        if action in ('accept', 'reject', 'pending'):
            entry['status'] = {'accept': 'accepted', 'reject': 'rejected', 'pending': 'pending'}[action]
        elif action == 'tag':
            entry['label'] = str(value or '')
            if entry['kind'] == 'mark':
                entry['effect'] = config.tag(entry['label'])['effect']
        elif action in ('note', 'outcome', 'text'):
            if action == 'outcome':
                entry['detail']['outcome'] = str(value or '')
            else:
                entry['note' if action == 'note' else 'label'] = str(value or '')
        elif action == 'split' and isinstance(value, (int, float)) and entry['t0'] < value < entry['t1']:
            tail = dict(entry, id=f"{entry['id']}/{round(value * 1000)}", t0=value, detail=dict(entry['detail']))
            entry['t1'] = value
            items[tail['id']] = tail
        elif action == 'merge' and value in items and value != entry['id']:
            other = items.pop(value)
            entry['t0'], entry['t1'] = min(entry['t0'], other['t0']), max(entry['t1'], other['t1'])
    return items


def conditions_at(conditions, t):
    """The water conditions set last at or before ``t``; before the first set, the first (set once per run)."""
    before = [c for at, c in conditions if at <= t]
    return before[-1] if before else (conditions[0][1] if conditions else None)


def label_spans(items):
    """``(t0, t1, effect, why)`` of everything that downgrades the windows it overlaps; rejected items don't."""
    spans = []
    for i in items:
        if i['status'] == 'rejected' or i['effect'] == 'none':
            continue
        if i['kind'] == 'mark':
            before, after = config.tag(i['label'])['span']
            spans.append((i['t0'] + before, i['t0'] + after, i['effect'], i['label']))
        else:
            spans.append((i['t0'], max(i['t1'], i['t0'] + 1.0), i['effect'], i['kind']))
    for i in items:
        if i['kind'] == 'fix' and FIX_EFFECT.get(i['label'], 'invalid') != 'none':
            spans.append((i['t0'], i['t1'], FIX_EFFECT.get(i['label'], 'invalid'), f"fix {i['label']}"))
    return spans


def quality(entry, spans):
    """``(worst effect, sorted reasons)`` of the spans overlapping ``entry``; 'ok' when none do."""
    hits = [(effect, why) for a, b, effect, why in spans if a < entry['t1'] and b > entry['t0']]
    worst = config.worst(effect for effect, _ in hits)
    return ('ok' if worst == 'none' else worst), sorted({why for _, why in hits})
