"""Header row 1 and the agent banner (ui spec 4.2): chips, deploy item, host, authority.

Pure Rich, no ROS. Authority is a solid chip carrying an upper-case word (ui spec section 1);
nothing else on the page uses a chip except ARMED and REVIEW. Optional view keys, all absent
today unless the app fills them:

  deploy     {ahead, behind, dirty, untracked, diverged, code_newer}
  authority  (kind, WORD) with kind in human ours nav agent hold unclaimed
  review     True replaces the authority and arm chips with a REVIEW chip
  agent      banner text; while set it replaces the health strip row
  agent_desk the AgentDesk view; its recent notes show ``◇ who 3s`` while no agent claim exists
"""

import time

from rich.text import Text

from mhseals_testing.ui import authority_chip, theme

BANNER_KEYS = 'E revoke' + theme.SEPARATOR + 'Space stop'
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
        item = Text(theme.pick('↕', '<>'), style=theme.BAD)
    else:
        marks = [(theme.pick('↑', '^'), 'ahead'), (theme.pick('↓', 'v'), 'behind'),
                 (theme.pick('±', '+-'), 'dirty'), ('?', 'untracked')]
        counts = ' '.join(f'{mark}{deploy[key]}' for mark, key in marks if deploy.get(key))
        item = (Text(counts, style=theme.WARN) if counts
                else Text(theme.pick('≡', '='), style=theme.MUTED))
    if deploy.get('code_newer'):
        item.append(' code newer', style=theme.WARN)
    return item


def host_item(view, short=False):
    review = view.get('review')
    if review and hasattr(review, 'title'):     # a past session: its title and host, no domain
        host = f'{review.title}{theme.SEPARATOR}{review.host}'.strip(' ' + theme.SEPARATOR)
        return Text(review.host if short else host, style=theme.MUTED)
    host = view['host'] if short else f"{view['host']}{theme.SEPARATOR}ROS {view['domain']}"
    return Text(host, style=theme.MUTED)


def working_item(view, now=None):
    """``◇ claude-3f2a 3s``: an agent is working on the boat but holds no claim; the glyph and age in
    muted magenta, the name in its vendor hue (Claude, Codex)."""
    seen = authority_chip.working(view.get('agent_desk'), view.get('authority_record'),
                                  time.time() if now is None else now)
    if not seen:
        return None
    return Text.assemble(f'{theme.WORKING_GLYPH} ', (seen[0], theme.agent_style(seen[0])),
                         f' {authority_chip.age(seen[1])}', style=theme.AGENT_DIM)


def chips(view):
    """The authority and arm chips, or the lone REVIEW chip."""
    if view.get('review'):
        return [theme.chip('REVIEW', 'hold')]
    kind, word = view.get('authority') or (None, None)
    out = [theme.chip(word, kind)] if word else [Text('idle', style=theme.MUTED)]
    out.append(theme.chip('ARMED', 'armed') if view['armed']
               else Text('disarmed', style=theme.MUTED))
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
                     style=f'bold {theme.ACCENT}' if name == current else theme.MUTED)
    return strip


def top_row(view, width):
    """Title, profile (wide), then right-aligned deploy, host, authority, arm; the first drops what does not fit."""
    title = Text.assemble((' boat_test ', f'bold {theme.ACCENT}'),
                          tab_strip(view.get('tab'), width))
    name = (f"{view['profile']}{theme.SEPARATOR}{view['units']}" if view.get('units')
            else view['profile'])
    profile = Text.assemble((f'  {name}', theme.MUTED))
    deploy, tail = deploy_item(view.get('deploy')), chips(view)
    work = None if view.get('review') else working_item(view)
    lefts = ([Text.assemble(title, profile)] if width >= PROFILE_AT else []) + [title]
    rights = [(deploy, work, host_item(view)), (deploy, work, host_item(view, True)),
              (None, work, host_item(view, True)), (None, work, None), (None, None, None)]
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


def _fit(text, room):
    """Drop the banner's lowest-ranked parts (pid and reason, then age) until it fits; else cut the middle."""
    parts = list(getattr(text, 'parts', ()))
    while parts and len(theme.SEPARATOR.join(t for t, _ in parts)) + 1 > room:
        worst = max(rank for _, rank in parts)
        if not worst:
            break
        parts.pop(max(i for i, (_, rank) in enumerate(parts) if rank == worst))
    return middle_cut(f" {theme.SEPARATOR.join(t for t, _ in parts) if parts else text}", room)


STOP_KEYS = 'U release' + theme.SEPARATOR + 'Space stop'


def stop_banner(armed, width):
    """`` ⊘ STOPPED · neutral sent · disarmed`` with the release and stop keys at the right.

    Only the glyph and STOPPED wear ``stop``; the rest is muted and nothing is reversed, so it
    does not shout over the page. The keys are never cut.
    """
    glyph = theme.pick(theme.PALETTE.glyph('stop', ascii=False), '(/)')
    head = f' {glyph} STOPPED'
    rest = middle_cut(f"{theme.SEPARATOR}neutral sent{theme.SEPARATOR}{'armed' if armed else 'disarmed'}",
                      max(width - len(head) - len(STOP_KEYS) - 2, 0))
    gap = max(width - len(head) - len(rest) - len(STOP_KEYS) - 1, 1)
    out = Text.assemble((head, f'bold {theme.PALETTE.style("stop")}'), (rest, theme.MUTED),
                        ' ' * gap, (STOP_KEYS, theme.MUTED), ' ', no_wrap=True)
    out.truncate(width)
    return out


def banner(text, width):
    """The whole row in the agent hue; the revoke and stop keys at the right are never cut.

    A low lease (``lease 3 s``) wears the warn cell so it reads before it lapses.
    """
    keys = BANNER_KEYS
    left = _fit(text, max(width - len(keys) - 2, 0))
    line = (left.ljust(max(width - len(keys) - 1, 0)) + keys + ' ')[:width]
    out = Text(line, style=theme.AGENT_BANNER, no_wrap=True)
    low = next((t for t, rank in getattr(text, 'parts', ()) if t.startswith('lease ') and not rank
                and t != 'lease ok'), None)
    if low and low in line:
        start = line.index(low)
        out.stylize(theme.AGENT_LOW, start, start + len(low))
    return out
