"""Header row 1 and the agent banner (ui spec 4.2): chips, deploy item, host, authority.

Pure Rich, no ROS. Authority is a solid chip carrying an upper-case word (ui spec section 1);
nothing else on the page uses a chip except ARMED and REVIEW. Optional view keys, all absent
today unless the app fills them:

  deploy     {ahead, behind, dirty, untracked, diverged, code_newer}
  authority  (kind, WORD) with kind in human ours nav agent hold unclaimed
  review     True replaces the authority and arm chips with a REVIEW chip
  agent      banner text; while set it replaces the health strip row
"""

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.theme import ACCENT, BAD, MUTED, SEPARATOR

BANNER_KEYS = 'E revoke' + SEPARATOR + 'Space stop'
PROFILE_AT = 100   # the profile shows from this width


def deploy_item(deploy):
    """``≡`` clean, counts in warn, ``↕`` diverged, ``code newer`` when the running code is stale.

    ``deploy`` is that dict, or the Text ``ui.deploy.deploy_text`` already built from the git probe.
    """
    if not deploy:
        return None
    if isinstance(deploy, Text):
        return deploy
    if deploy.get('diverged'):
        item = Text(theme.pick('↕', '<>'), style=BAD)
    else:
        marks = [(theme.pick('↑', '^'), 'ahead'), (theme.pick('↓', 'v'), 'behind'),
                 (theme.pick('±', '+-'), 'dirty'), ('?', 'untracked')]
        counts = ' '.join(f'{mark}{deploy[key]}' for mark, key in marks if deploy.get(key))
        item = Text(counts, style=theme.WARN) if counts else Text(theme.pick('≡', '='), style=MUTED)
    if deploy.get('code_newer'):
        item.append(' code newer', style=theme.WARN)
    return item


def host_item(view, short=False):
    review = view.get('review')
    if review and hasattr(review, 'title'):     # a past session: its title and host, no domain
        host = f'{review.title}{SEPARATOR}{review.host}'.strip(' ' + SEPARATOR)
        return Text(review.host if short else host, style=MUTED)
    host = view['host'] if short else f"{view['host']}{SEPARATOR}ROS {view['domain']}"
    return Text(host, style=MUTED)


def chips(view):
    """The authority and arm chips, or the lone REVIEW chip."""
    if view.get('review'):
        return [theme.chip('REVIEW', 'hold')]
    kind, word = view.get('authority') or (None, None)
    out = [theme.chip(word, kind)] if word else [Text('idle', style=MUTED)]
    out.append(theme.chip('ARMED', 'armed') if view['armed'] else Text('disarmed', style=MUTED))
    return out


def join(items, gap='  '):
    return Text(gap).join([item for item in items if item is not None])


def tab_strip(current, width=120):
    """``[Drive]  Correction  Map  Bags``: the current tab bracketed and bold, the others muted.

    Below 80 columns only the current tab shows (ui.md 4.2).
    """
    from mhseals_testing.ui.layout import TABS   # layout imports chrome
    current = current or TABS[0]
    strip = Text()
    for name in TABS if width >= 80 else (current,):
        strip.append(f' [{name}]' if name == current else f'  {name}',
                     style=f'bold {ACCENT}' if name == current else MUTED)
    return strip


def top_row(view, width):
    """Title, profile (wide), then right-aligned deploy, host, authority, arm; the first drops what does not fit."""
    title = Text.assemble((' boat_test ', f'bold {ACCENT}'), tab_strip(view.get('tab'), width))
    name = f"{view['profile']}{SEPARATOR}{view['units']}" if view.get('units') else view['profile']
    profile = Text.assemble((f'  {name}', MUTED))
    deploy, tail = deploy_item(view.get('deploy')), chips(view)
    lefts = ([Text.assemble(title, profile)] if width >= PROFILE_AT else []) + [title]
    rights = [(deploy, host_item(view)), (deploy, host_item(view, True)), (None, host_item(view, True)),
              (None, None)]
    for left in lefts:
        for items in rights:
            right = join([*items, *tail])
            gap = width - len(left) - len(right)
            if gap >= 1:
                return Text.assemble(left, ' ' * gap, right)
    return Text.assemble(title, ' ', join(tail))


def middle_cut(text, width):
    if len(text) <= width:
        return text
    ellipsis = theme.pick('…', '...')
    keep = max(width - len(ellipsis), 0)
    return text[:keep - keep // 2] + ellipsis + text[len(text) - keep // 2:] if keep else ellipsis[:width]


def banner(text, width):
    """The whole row in the agent hue; the revoke and stop keys at the right are never cut."""
    keys = BANNER_KEYS
    left = middle_cut(f' {text}', max(width - len(keys) - 2, 0))
    line = left.ljust(max(width - len(keys) - 1, 0)) + keys + ' '
    return Text(line[:width], style=theme.AGENT_BANNER, no_wrap=True)
