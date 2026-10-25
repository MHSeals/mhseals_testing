#!/usr/bin/env python3
"""Render the glyph and arm-panel ideas through the TUI's own Rich path into one HTML gallery.

    python3 tools/glyph_gallery.py OUT.html

Each tier (truecolor, NO_COLOR, ASCII) is rendered in its own process, because theme reads the
environment at import; the parent merges the fragments.
"""

import html
import io
import json
import os
import subprocess
import sys

HEADINGS = (0, 22.5, 45, 67.5, 90, 135, 180, 270)
ALL16 = tuple(k * 22.5 for k in range(16))
TODAY = ('0', 'T', 'U', 'S')
BOATS = ('0', 'A', 'B', 'C', 'D', 'E')
ARROWS = ('0', 'A', 'B', 'C', 'D', 'E')
TODAY_NOTES = (
    ('0', 'Today', 'Barbs at ±35° rounded per dot: clean at 0/45/90, ragged at 22.5° and 67.5°.'),
    ('T', 'Hand-drawn wedge, 6 dots', 'A filled right-angle wedge (edges at 180° and 225° for 22.5°, '
     'both clean dot lines) drawn once and mapped to the other seven by the square’s flips and transpose, '
     'which are exact on square dots. The shaft stops 3 dots short and joins the wedge’s base. A table of '
     '6 offsets; no per-frame cost beyond today’s.'),
    ('U', 'Hand-drawn wedge, 10 dots', 'As T with a fourth row: a bigger head than today’s 0°.'),
    ('S', 'Snapped head', 'Today’s head drawn at the nearer cardinal while the shaft stays exact: '
     'tidy dots, but the head visibly points 22.5° off the shaft.'),
)
COMBOS = (('CE', 'C hull + E glow arrow'), ('EE', 'E chevron + E glow arrow'), ('DA', 'D hybrid + A bold arrow'), ('CC', 'C hull + C half-block arrow'))
ARMS = ('0', '1', '2', '3')
TIERS = {
    'truecolor': {'COLORTERM': 'truecolor', 'TERM': 'xterm-256color'},
    'mono': {'NO_COLOR': '1', 'TERM': 'xterm-256color'},
    'ascii': {'MHS_ASCII': '1', 'TERM': 'xterm-256color', 'COLORTERM': 'truecolor'},
}
FG, BG = '#D5DBE5', '#171B22'


# Block elements drawn as CSS rectangles (x, y, w, h in %), the way kitty, WezTerm, foot and
# Ghostty draw them natively: no font seams. Braille and the rest stay font glyphs.
_Q = {1: (0, 0, 50, 50), 2: (50, 0, 50, 50), 4: (0, 50, 50, 50), 8: (50, 50, 50, 50)}
BLOCKS = {ch: [_Q[b] for b in (1, 2, 4, 8) if i & b] for i, ch in enumerate(' ▘▝▀▖▌▞▛▗▚▐▜▄▙▟█') if i}
BLOCKS.update({'▀': [(0, 0, 100, 50)], '▄': [(0, 50, 100, 50)], '█': [(0, 0, 100, 100)],
               '▌': [(0, 0, 50, 100)], '▐': [(50, 0, 50, 100)], '▏': [(0, 0, 12.5, 100)],
               '▕': [(87.5, 0, 12.5, 100)], '▁': [(0, 87.5, 100, 12.5)], '▔': [(0, 0, 100, 12.5)]})
for n, ch in enumerate('▎▍▋▊▉', 2):
    BLOCKS[ch] = [(0, 0, {2: 25, 3: 37.5, 4: 62.5, 5: 75, 6: 87.5}[n], 100)]


def _cell(ch, fg, bg, css):
    style = list(css)
    if ch in BLOCKS:
        colour = fg or FG
        layers = []
        for x, y, w, h in BLOCKS[ch]:
            px = 0 if w == 100 else x / (100 - w) * 100
            py = 0 if h == 100 else y / (100 - h) * 100
            layers.append(f'linear-gradient({colour},{colour}) {px:g}% {py:g}%/{w:g}% {h:g}% no-repeat')
        style.append('background:' + ','.join(layers + ([bg] if bg else [])))
        return f'<i style="{";".join(style)}"></i>'
    if fg:
        style.append(f'color:{fg}')
    if bg:
        style.append(f'background:{bg}')
    if ch in (' ', '\u2800') and not bg:
        return '<i></i>'
    return f'<i style="{";".join(style)}">{html.escape(ch)}</i>' if style else f'<i>{html.escape(ch)}</i>'


def to_html(renderable, width):
    from rich.console import Console
    console = Console(width=width, file=io.StringIO(), color_system='truecolor', force_terminal=True,
                      legacy_windows=False)
    out = []
    for line in console.render_lines(renderable, console.options.update(width=width), pad=False):
        parts = []
        for seg in line:
            if not seg.text or seg.control:
                continue
            st = seg.style
            fg = bg = None
            css = []
            if st is not None:
                if st.color is not None and not st.color.is_default:
                    fg = st.color.get_truecolor().hex
                if st.bgcolor is not None and not st.bgcolor.is_default:
                    bg = st.bgcolor.get_truecolor().hex
                if st.reverse:
                    fg, bg = (bg or BG), (fg or FG)
                if st.bold:
                    css.append('font-weight:700')
                if st.dim:
                    css.append('opacity:.55')
                if st.underline:
                    css.append('text-decoration:underline')
            parts.extend(_cell(ch, fg, bg, css) for ch in seg.text)
        out.append('<b>' + ''.join(parts) + '</b>')
    return ''.join(out)


def dump(tier):
    """{key: html} for one tier."""
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
    from rich.console import Group
    from mhseals_testing.ui import glyph_ideas as gi, arm_ideas
    out = {}
    for opt in BOATS:
        for h in HEADINGS:
            out[f'boat/{opt}/{h}'] = to_html(Group(*gi.tile(opt, h)), 9)
            out[f'boatrose/{opt}/{h}'] = to_html(Group(*gi.rose(boat=opt, arrow_opt='0', heading=h,
                                                                command_bearing=None, drift=None)), 21)
    for opt in ARROWS:
        for h in HEADINGS:
            out[f'arrow/{opt}/{h}'] = to_html(Group(*gi.rose(boat='0', arrow_opt=opt, heading=0,
                                                             command_bearing=h)), 21)
    for opt in TODAY:
        for h in ALL16:
            out[f'today/{opt}/{h}'] = to_html(Group(*gi.rose(boat='0', arrow_opt=opt, heading=0,
                                                             command_bearing=h, drift=None)), 21)
    for boat, arrow in ((k[0], k[1]) for k, _ in COMBOS):
        for h in HEADINGS:
            out[f'combo/{boat}{arrow}/{h}'] = to_html(Group(*gi.rose(boat=boat, arrow_opt=arrow, heading=h,
                                                                     command_bearing=h + 30, magnitude=0.7)), 21)
    for opt in ARMS:
        for state in arm_ideas.STATES:
            for width in (80, 120):
                out[f'arm/{opt}/{state}/{width}'] = to_html(arm_ideas.render(opt, state, width), width)
    return out


# --- page -------------------------------------------------------------------------------------

BOAT_NOTES = {
    '0': ('Today: braille hull mask', '9 × 6 dots, point-in-polygon per 3°, cached. Reads as a stipple; '
          'jaggy at 22.5° and 67.5°, the bow hue lands on 2–3 loose dots.', 'braille only'),
    'A': ('Quadrant hull, 16 headings', 'Same hull, rasterised once per 22.5° to quadrant blocks '
          '(▖▗▘▝▙▛▜▟▚▞): solid, no dot gaps. Table of 16 × ≤12 cells, built at import; a frame '
          'copies ≤12 cells. One hue per cell (bow cells accent). Works in 256/16 colours and NO_COLOR.',
          'Block Elements U+2596–259F: in every common mono font (DejaVu, Menlo, Consolas, JetBrains, '
          'Cascadia). Low risk.'),
    'B': ('Pointer: triangle + stern, 8 headings', 'Hand-placed table, 8 entries of 2–4 cells; '
          'zero compute. Very readable as a direction, but only 45° steps and it no longer looks like a hull.',
          'Geometric Shapes ▲▶▼◀ safe; ◢◣◤◥ are narrower/heavier than blocks in some fonts '
          '(Consolas, Menlo falls back). Medium risk on diagonals.'),
    'C': ('Half-block hull, antialiased', '1 × 2 square pixels per cell; in truecolor each pixel '
          'blends its coverage into the panel bg, the cell carries 2 colours (fg ▀ on bg). '
          'Precomputed per 5° (72 entries of ≤14 cells, ~1 ms each at first use). 256/16/NO_COLOR: '
          'thresholded, no blend. Sets a cell bg, so it paints over costmap tints on the map.',
          '▀▄█ only: lowest font risk. Needs a font whose blocks fill the line height (most do; '
          'line spacing >1.0 in the terminal shows thin seams).'),
    'E': ('Own-ship chevron, antialiased', 'A notched chevron (the chart-plotter own-ship mark) in '
          'the same AA half blocks as C; per 5°, ≤12 cells. Too small for the notch to survive: at 22.5°/67.5° '
          'it breaks into blobs. Rejected.',
          '▀▄█ only: lowest risk.'),
    'D': ('Hybrid: quadrant body + braille bowsprit', 'A’s 16-heading body plus a 3-dot braille '
          'line at the exact heading (per 5°): body reads at a glance, the bowsprit gives the '
          'fine angle. Same cost as A plus 3 dots.', 'Block Elements + braille: low risk.'),
}
ARROW_NOTES = {
    '0': ('Today (compass-hires): braille line + braille barbs', '1-dot shaft, two 3-dot barbs. '
          'Thin; the head blurs into the shaft at 22.5°/67.5°.', 'braille only'),
    'A': ('Bold braille: 2-dot shaft, filled wedge head', 'Shaft offset ±½ dot (2 dots wide) at '
          '62 % of the hue, a filled 5 × 6-dot triangle head at full hue. Any bearing; cached per '
          'degree and length (≈80 dots).', 'braille only: low risk.'),
    'B': ('Geometric head glyph', 'Braille shaft and a ▲◥▶◢▼◣◀◤ cell at the tip. Bold and '
          'crisp at 0/45/90…, but the head snaps to 45°: at 22.5° and 67.5° it visibly '
          'disagrees with the shaft.', '◢◣◤◥ width/weight vary by font: medium risk.'),
    'C': ('Half-block silhouette, AA + gradient', 'Shaft 1 unit wide and a 3-unit head as one '
          'polygon, rasterised to half blocks; truecolor blends coverage into the bg and fades '
          'the shaft 45 %→100 % toward the head. Cached per degree and length (~2 ms first time). '
          'Below truecolor: solid thresholded blocks.', '▀▄█ only: lowest risk.'),
    'E': ('Glow: gradient shaft, wedge head, tinted cells', '1-dot braille shaft fading 50 %→100 %, '
          'a filled wedge head in bold hue on a 20 % tint of the hue (the head cell or two): the tip reads '
          'as lit, the rose stays braille. A tinted shaft was tried and dropped: the cell-sized tint '
          'reads as blocky rectangles. Cached per degree.', 'braille only: low risk. Tint and gradient are '
          'truecolor/256 only; 16 colours and NO_COLOR draw A’s shape flat.'),
    'D': ('Comet: gradient shaft, wedge, fletching', '1-dot shaft fading 35 %→100 % per cell, '
          'filled wedge head, two swept feathers at the tail. Any bearing; cached per degree.',
          'braille only: low risk. Gradient is truecolor-only (256: nearest steps; 16/NO_COLOR: flat).'),
}
ARM_NOTES = {
    '0': ('Today', 'Bold accent labels, rules per group, danger (fail) border, slots field.'),
    '1': ('Quiet labels, state gutter', 'Labels muted and right-aligned to one column, values in fg; '
          'each check has a ✓/✗/! gutter in its state colour; checks grouped as blocking vs clear '
          'with a count on the rule; the caution line in warn; one blank line before the action.'),
    '2': ('Armed-hue frame', 'Border and title in the armed token (the state you are entering), '
          'not fail; the outputs value is the one emphasised value (“real thrusters”); '
          'checks as a two-column checklist; the field sits under a “type ARM” label.'),
    '3': ('Compact card', 'Groups as small headings instead of full-width rules, 1-cell indent, '
          'values left-aligned at one tab stop; blockers first and in fail, clear checks collapsed to '
          'one line; focus chip unchanged.'),
}


def page(frags):
    def pre(key, tier='truecolor'):
        return f'<pre class="t">{frags[tier].get(key, "")}</pre>'

    def strip(label, keyfmt, tier='truecolor', labels=True, headings=HEADINGS):
        cells = ''.join(f'<div>{f"<span>{h:g}°</span>" if labels else ""}{pre(keyfmt.format(h=h), tier)}</div>'
                        for h in headings)
        return f'<div class="row"><span class="k">{label}</span><div class="strip">{cells}</div></div>'

    parts = [HEAD]
    parts.append('<h1>Boat glyph, command arrow and arm panel ideas</h1>'
                 '<p class="lede">Rendered by the TUI’s own Rich path (<code>mhseals_testing/ui/glyph_ideas.py</code>, '
                 '<code>arm_ideas.py</code> and <code>tools/glyph_gallery.py</code> on branch <code>glyph-ideas</code>) '
                 'with the dark palette of <code>config/theme.yaml</code>, then copied cell for cell: 9 × 18 px '
                 'cells, block elements drawn as exact rectangles the way kitty, WezTerm, foot and Ghostty draw '
                 'them (a font-drawn terminal may show hairline seams). Braille and shapes are font glyphs.</p>')

    parts.append('<h2>0 · Owner’s arrow pick: today’s arrow, in-between bearings fixed</h2>'
                 '<p class="lede">All 16 bearings, heading 0, command at 80 %. 0°, 45°, 90° … are drawn exactly '
                 'as today in every row; only 22.5° + 45° k change. The shaft is always exact.</p>')
    for opt, title, note in TODAY_NOTES:
        parts.append(f'<section><h3><b>{opt}</b> {title}</h3>'
                     + strip('truecolor', f'today/{opt}/{{h}}', headings=ALL16)
                     + (strip('NO_COLOR', f'today/{opt}/{{h}}', 'mono', labels=False, headings=ALL16) if opt == 'T' else '')
                     + f'<p class="note">{note}</p></section>')
    parts.append('<section class="rec"><h3>Recommendation</h3><p class="note">'
                 '<b>Boat: C</b>, the antialiased half-block hull: solid rather than stippled, any heading '
                 '(cached per 5°), ▀▄█ only, thresholded below truecolor. On the map its edge pixels should blend '
                 'into the cell’s own costmap tint rather than the panel bg. Runner-up: D, if the owner wants '
                 'the exact heading as a bowsprit.<br>'
                 '<b>Arrow (owner picked today’s): T</b>, a hand-drawn 6-dot wedge for 22.5° mirrored to all eight '
                 'in-between bearings, the same 3-row size and filled look as today’s 0° head. U is the larger '
                 '10-dot wedge if T reads small; S (snapped head) visibly kinks against the shaft.<br>'
                 '<b>Arm panel: 1</b>, quiet labels with a state gutter: muted right-aligned labels, values in fg, '
                 'checks with ✓/✗ in state colour under a rule that counts what blocks, the caution line in warn; '
                 'the danger border and title stay fail. Variant 2 is the bolder pick (armed-hue frame).</p></section>')
    parts.append('<h2>1 · Boat glyph</h2><p class="lede">Each option alone (9 × 5 cells) and in the '
                 'Conditions rose. Headings are screen bearings, clockwise from up.</p>')
    for opt in BOATS:
        title, cost, font = BOAT_NOTES[opt]
        parts.append(f'<section><h3><b>{opt}</b> {title}</h3>')
        parts.append(strip('alone', f'boat/{opt}/{{h}}'))
        parts.append(strip('in the rose', f'boatrose/{opt}/{{h}}', labels=False))
        parts.append(strip('NO_COLOR', f'boat/{opt}/{{h}}', 'mono', labels=False))
        parts.append(strip('ASCII', f'boat/{opt}/{{h}}', 'ascii', labels=False))
        parts.append(f'<p class="note"><b>Cost</b> {cost}<br><b>Font</b> {font}</p></section>')

    parts.append('<h2>2 · Command arrow</h2><p class="lede">North-up rose, heading 0, the command '
                 'arrow at each bearing (80 % of full), today’s hull so only the arrow varies. '
                 'Hue: the accent (a task drives).</p>')
    for opt in ARROWS:
        title, cost, font = ARROW_NOTES[opt]
        parts.append(f'<section><h3><b>{opt}</b> {title}</h3>')
        parts.append(strip('truecolor', f'arrow/{opt}/{{h}}'))
        parts.append(strip('NO_COLOR', f'arrow/{opt}/{{h}}', 'mono', labels=False))
        parts.append(f'<p class="note"><b>Cost</b> {cost}<br><b>Font</b> {font}</p></section>')
    parts.append('<section><h3>ASCII, every option</h3><p class="note">ASCII keeps today’s rose: a '
                 '<code>.</code> line and an <code>^ / &gt; \\</code> head glyph.</p>'
                 + strip('ASCII', 'arrow/0/{h}', 'ascii') + '</section>')

    parts.append('<h2>3 · Together</h2><p class="lede">Heading turning, command 30° to starboard '
                 'of the bow at 70 %, drift east-north-east.</p>')
    for key, label in COMBOS:
        parts.append(f'<section><h3>{label}</h3>' + strip('truecolor', f'combo/{key}/{{h}}')
                     + strip('NO_COLOR', f'combo/{key}/{{h}}', 'mono', labels=False) + '</section>')

    parts.append('<h2>4 · Arm panel</h2><p class="lede">The <code>a</code> popup through '
                 '<code>layout.popup</code>: typed ARM on the boat (two letters typed), a blocked arm on '
                 'the sim (Cancel focused), and the armed panel (Disarm / Close). Every variant keeps the '
                 'structure, the typed word, the Cancel default, the refusal rows and the danger cue; only '
                 'colour, alignment, spacing and grouping change.</p>')
    for opt in ARMS:
        title, what = ARM_NOTES[opt]
        parts.append(f'<section><h3><b>{opt}</b> {title}</h3><p class="note">{what}</p>')
        for state in ('typed', 'blocked', 'armed'):
            parts.append(f'<h4>{state}</h4><div class="strip">'
                         f'<div><span>truecolor · 80 cols</span>{pre(f"arm/{opt}/{state}/80")}</div>'
                         f'<div><span>NO_COLOR</span>{pre(f"arm/{opt}/{state}/80", "mono")}</div>'
                         f'<div><span>ASCII</span>{pre(f"arm/{opt}/{state}/80", "ascii")}</div></div>')
        parts.append(f'<h4>typed · 120 cols</h4>{pre(f"arm/{opt}/typed/120")}</section>')
    parts.append('</body></html>')
    return '\n'.join(parts)


HEAD = f'''<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Glyph Ideas Gallery</title>
<style>
:root {{ --bg:{BG}; --fg:{FG}; --muted:#7C8595; --border:#3E4656; --accent:#7DD3DC; --sel:#232A36; }}
body {{ background:var(--bg); color:var(--fg); margin:0; padding:24px 16px 64px;
  font-family:"JetBrains Mono","DejaVu Sans Mono","Cascadia Mono",Menlo,Consolas,"Noto Sans Mono",monospace;
  font-size:14px; line-height:1.45; }}
pre.t, pre.t * {{ font-size:15px; }}
h1 {{ font-size:20px; margin:0 0 8px; color:var(--accent); }}
h2 {{ font-size:16px; margin:40px 0 4px; color:var(--accent); border-bottom:1px solid var(--border); padding-bottom:4px; }}
h3 {{ font-size:14px; margin:20px 0 6px; font-weight:600; }}
h3 b {{ display:inline-block; min-width:1.6em; color:var(--bg); background:var(--accent); text-align:center;
  border-radius:4px; margin-right:8px; }}
h4 {{ font-size:12px; color:var(--muted); margin:12px 0 4px; font-weight:400; }}
.lede, .note {{ color:var(--muted); max-width:110ch; margin:4px 0 10px; }}
.note b {{ color:var(--fg); font-weight:600; }}
code {{ color:var(--fg); }}
section {{ border:1px solid var(--border); border-radius:10px; padding:4px 14px 10px; margin:14px 0; overflow-x:auto; }}
table {{ border-collapse:collapse; }}
th {{ color:var(--muted); font-weight:400; font-size:12px; text-align:left; padding:2px 10px 2px 0; }}
td {{ vertical-align:top; padding:4px 14px 4px 0; }}
td.k, span.k {{ color:var(--muted); font-size:12px; white-space:nowrap; display:block; }}
pre.t {{ margin:0; font:inherit; color:var(--fg); }}
pre.t b {{ display:block; height:18px; white-space:nowrap; font-weight:400; }}
pre.t i {{ display:inline-block; width:9px; height:18px; line-height:18px; font-style:normal;
  text-align:center; vertical-align:top; overflow:visible; }}
.strip {{ display:flex; flex-wrap:wrap; gap:6px 18px; align-items:flex-start; margin:4px 0 8px; }}
.strip > div > span {{ color:var(--muted); font-size:11px; display:block; }}
.row {{ display:flex; gap:12px; align-items:flex-start; }}
.row > .k {{ width:80px; flex:none; padding-top:16px; }}
section.rec {{ border-color:var(--accent); }}
.arm {{ display:flex; flex-wrap:wrap; gap:18px; }}
</style></head><body>'''


def main():
    if len(sys.argv) == 3 and sys.argv[1] == '--dump':
        json.dump(dump(sys.argv[2]), sys.stdout)
        return
    out = sys.argv[1]
    frags = {}
    for tier, env in TIERS.items():
        clean = {k: v for k, v in os.environ.items() if k not in ('NO_COLOR', 'MHS_ASCII', 'CX_ASCII', 'COLORTERM')}
        clean.update(env)
        result = subprocess.run([sys.executable, __file__, '--dump', tier], env=clean, check=True,
                                capture_output=True, text=True)
        frags[tier] = json.loads(result.stdout)
    with open(out, 'w', encoding='utf-8') as handle:
        handle.write(page(frags))
    print(out)


if __name__ == '__main__':
    main()
