"""Inspect section 5: RC rate and latency, and any other advisory's number over time."""

from rich.text import Text

from mhseals_testing.ui import sparks, theme
from mhseals_testing.ui.popups.inspect.common import SEP, duration, kv, recent, strip


# --- 5. RC and advisories ---------------------------------------------------------------------

def rc_part(state, snapshot, inner):
    rc = (snapshot.get('control') or {}).get('rc')
    if not rc:
        return [kv('RC', Text('no RC_CHANNELS seen', style=theme.MUTED))]
    out = [kv('Mode', rc.get('mode') or theme.DASH, SEP, 'valid' if rc.get('valid', True) else 'lost')]
    out += strip('Rate', recent(state.series.get('rate', ()), state.now), state.now, inner, 'Hz')
    out += strip('Latency', recent(state.series.get('latency', ()), state.now), state.now, inner, 'ms')
    for i, us in enumerate((rc.get('channels') or [])[:8]):
        out.append(kv(f'ch{i + 1}', sparks.hbar(us, 1000, 2000, 30), f"  {us} {theme.pick('µs', 'us')}"))
    return out


def advisory_part(state, snapshot, inner):
    adv = snapshot.get('advisories', {}).get(state.row['name'])
    out = []
    if adv:
        out.append(kv('Level', theme.labelled(adv['level']), f'{SEP}updated {duration(adv["age"])} ago'))
    if state.series.get('value'):
        out += strip('Value', recent(state.series['value'], state.now), state.now, inner, '')
    return out
