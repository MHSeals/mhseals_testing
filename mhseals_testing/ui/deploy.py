"""header.deploy: one glyph group for how far the checkouts are from origin."""

from rich.text import Text

from mhseals_testing.ui import theme

STALE = 3600.0           # a fetch older than this shows its age


def _age(seconds):
    for unit, size in (('d', 86400), ('h', 3600), ('m', 60)):
        if seconds >= size:
            return f'{int(seconds // size)}{unit}'
    return f'{int(seconds)}s'


def deploy_text(states, stale=STALE, code_newer=False):
    """Sum the git_state ``Result.data`` dicts: ``≡`` clean, else ``↑ahead ↓behind ±modified ?untracked``.

    A red ``↕`` marks a diverged repo, and a dim ``↻age`` the oldest fetch when it is stale.
    ``code_newer`` appends ``code newer``: the running code is older than the checkout.
    Returns None without states (the probe has not run).
    """
    states = [s for s in states if s]
    if not states:
        return None
    total = {name: sum(s.get(name, 0) for s in states)
             for name in ('ahead', 'behind', 'dirty', 'untracked')}
    glyphs = (('ahead', theme.pick('↑', '^')), ('behind', theme.pick('↓', 'v')),
              ('dirty', theme.pick('±', '*')), ('untracked', '?'))
    text = Text()
    if any(s.get('diverged') for s in states):
        text.append(theme.pick('↕', '<>'), style=theme.BAD)
    parts = [f'{glyph}{total[name]}' for name, glyph in glyphs if total[name]]
    if parts:
        text.append((' ' if text else '') + ' '.join(parts), style=theme.WARN)
    elif not text:
        text.append(theme.pick('≡', '='), style=theme.MUTED)
    ages = [s['fetch_age'] for s in states if s.get('fetch_age') is not None]
    if ages and max(ages) > stale:
        text.append(f' {theme.pick("↻", "~")}{_age(max(ages))}', style=theme.MUTED)
    if code_newer:
        text.append(' code newer', style=theme.WARN)
    return text
