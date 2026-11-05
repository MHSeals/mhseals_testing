"""header.deploy: one glyph group for how far the checkouts are from origin (model/deploy.py's Deploy)."""

from rich.text import Text

from mhseals_testing.model.deploy import STALE, age as _age, summary  # noqa: F401  (STALE re-export)
from mhseals_testing.ui import theme


def render(deploy):
    """The Text of a model ``Deploy``: ``≡`` clean, else ``↑ahead ↓behind ±modified ?untracked``; None for None.

    A red ``↕`` marks a diverged repo, and a dim ``↻age`` the oldest fetch when it is stale.
    ``code_newer`` appends ``code newer``: the running code is older than the checkout.
    """
    if deploy is None:
        return None
    total = deploy.counts
    glyphs = (('ahead', theme.pick('↑', '^')), ('behind', theme.pick('↓', 'v')),
              ('dirty', theme.pick('±', '*')), ('untracked', '?'))
    text = Text()
    if deploy.diverged:
        text.append(theme.pick('↕', '<>'), style=theme.BAD)
    parts = [f'{glyph}{total[name]}' for name, glyph in glyphs if total[name]]
    if parts:
        text.append((' ' if text else '') + ' '.join(parts), style=theme.WARN)
    elif not text:
        text.append(theme.pick('≡', '='), style=theme.MUTED)
    if deploy.stale_age is not None:
        text.append(f' {theme.pick("↻", "~")}{_age(deploy.stale_age)}', style=theme.MUTED)
    if deploy.code_newer:
        text.append(' code newer', style=theme.WARN)
    return text


def deploy_text(states, stale=STALE, code_newer=False):
    """Sum the git_state ``Result.data`` dicts and render them (``render(summary(...))``); None without states."""
    return render(summary(states, stale, code_newer))
