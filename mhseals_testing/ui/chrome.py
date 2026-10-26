"""Header row 1 and the agent banner (ui spec 4.2): chips, deploy item, host, authority.

Pure Rich, no ROS. Authority is a solid chip carrying an upper-case word (ui spec section 1);
nothing else on the page uses a chip except ARMED and REVIEW. Optional view keys, all absent
today unless the app fills them:

  deploy     {ahead, behind, dirty, untracked, diverged, code_newer}
  authority  (kind, WORD) with kind in human ours nav agent hold unclaimed
  review     True replaces the authority and arm chips with a REVIEW chip
  agent      banner text; while set it replaces the health strip row (and joins the STOPPED row)
  armed_by   None for the operator's arm, else {'who', 'sim'}: an agent's arm, marked after ARMED
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


def chips(view, short=False):
    """The authority and arm chips, or the lone REVIEW chip; ``short`` drops the arming agent's name."""
    if view.get('review'):
        return [theme.chip('REVIEW', 'hold')]
    kind, word = view.get('authority') or (None, None)
    out = [theme.chip(word, kind)] if word else [Text('idle', style=theme.MUTED)]
    out.append(theme.chip('ARMED', 'armed') if view['armed']
               else Text('disarmed', style=theme.MUTED))
    if view['armed'] and view.get('armed_by'):
        out.append(armed_by(view['armed_by'], short))
    return out


def armed_by(by, short=False):
    """``◆ claude sim-arm``: an agent armed, not the operator; its hue is the agent's, not the chip's.
    ``short`` (a narrow row, where the AGENT chip already names who) keeps ``◆ sim-arm``."""
    who = None if short else by.get('who') or 'agent'
    return Text(_words(theme.AGENT_GLYPH, who, 'sim-arm' if by.get('sim') else ''),
                style=f'bold {theme.PALETTE.style("auth.agent")}'.strip())


def _words(*parts):
    return ' '.join(str(p) for p in parts if p)


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
    deploy, tails = deploy_item(view.get('deploy')), (chips(view), chips(view, short=True))
    work = None if view.get('review') else working_item(view)
    lefts = ([Text.assemble(title, profile)] if width >= PROFILE_AT else []) + [title]
    rights = [(deploy, work, host_item(view)), (deploy, work, host_item(view, True)),
              (None, work, host_item(view, True)), (None, work, None), (None, None, None)]
    for tail in tails:
        for left in lefts:
            for items in rights:
                right = join([*items, *tail])
                gap = width - len(left) - len(right)
                if gap >= 1:
                    return Text.assemble(left, ' ' * gap, right)
    return Text.assemble(title, ' ', join(tails[-1]))


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
AGENT_STOP_KEYS = 'U release' + theme.SEPARATOR + 'E revoke' + theme.SEPARATOR + 'Space stop'


def stop_banner(armed, width, agent=None):
    """`` ⊘ STOPPED · neutral sent · disarmed`` with the release and stop keys at the right.

    Only the glyph and STOPPED wear ``stop``; the rest is muted and nothing is reversed, so it
    does not shout over the page. The keys are never cut. With an agent claim (``agent`` is its
    banner text) the row keeps STOPPED first and adds the claim in the agent hue, unreversed, so
    stopped still wins the eye; E revoke joins the keys.
    """
    keys = AGENT_STOP_KEYS if agent else STOP_KEYS
    glyph = theme.pick(theme.PALETTE.glyph('stop', ascii=False), '(/)')
    head = f' {glyph} STOPPED'
    room = max(width - len(head) - len(keys) - 2, 0)
    sent = '' if agent else f'{theme.SEPARATOR}neutral sent'    # the claim takes its cells
    rest = middle_cut(f"{sent}{theme.SEPARATOR}{'armed' if armed else 'disarmed'}", room)
    left = room - len(rest) - len(theme.SEPARATOR)
    claim = theme.SEPARATOR + stop_claim(agent, left + 1) if agent and left > 2 else ''
    gap = max(width - len(head) - len(rest) - len(claim) - len(keys) - 1, 1)
    out = Text.assemble((head, f'bold {theme.PALETTE.style("stop")}'), (rest, theme.MUTED),
                        (claim, theme.PALETTE.style('auth.agent')), ' ' * gap, (keys, theme.MUTED), ' ',
                        no_wrap=True)
    out.truncate(width)
    return out


def stop_claim(agent, room):
    """The agent claim on the STOPPED row: the banner's words, else ``◆ who · c7`` before a middle cut."""
    full = _fit(agent, room).lstrip()
    parts = list(getattr(agent, 'parts', ()))
    if theme.pick('…', '...') not in full or len(parts) < 2:
        return full
    short = authority_chip.Banner()
    short.parts = [(f'{theme.AGENT_GLYPH} {parts[1][0]}', 0), *parts[2:]]
    return _fit(short, room).lstrip()


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
