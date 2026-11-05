# The TUI in detail

The README's key table and `?` are the reference for keys; this is the rest (moved from the README, which
stays short: D-20261101-26).

## Controls and look

Hold-to-confirm: the confirms config/input.yaml `hold:` names (take control `O`, quitting while armed; arm and
release after the 11-04 pool test) fill a bar while Enter, the action's key or a mouse button is held; a tap or Esc
cancels and Space still stops at once. With `mouse: true` the wheel scrolls lists and popups; manual driving ignores it.
Held-key combos (Conditions reads `keys·combos`, else `keys·one key`) need the Kitty keyboard protocol (Kitty,
Ghostty, foot, WezTerm). Speed has one palette (spd.*): the surge, sway and yaw bars, the manual speed gauge, the
Conditions `sog` readout and the minimap trail run calm grey to hot magenta. The Conditions ring glows red toward the nearest lidar point
(/perception/nearby) inside 8 m, wider and bold inside 3 m; past 0.15 m/s a dashed wake trails off the stern, longer
with speed. Glyphs carry every state
(✓ ok, ! warn, ✗ fail, ◌ busy, · off), so `NO_COLOR` reads; `TERM=dumb` or `MHS_ASCII=1` draws ASCII.
Status lists the parts Enter starts in their own **Stack** section: a spinner while one starts, ✓ running, ✗ exited,
and `▶` after the name where Enter would start it.
The focused panel's title is a solid accent bar (` ▸ Status `, reverse video), so focus shows at a glance.

## Procedures

Each row wears its state: ▷ ready, ⊘ blocked with the reason on the row, a spinner (and the percent of a long
run) while running, ✓ or ✗ for its last result this session; the focused panel's bottom border is the legend.

## Backend

`mhseals_testing/backend/` is the link of the coming UI-less backend (D-20261101-19): a versioned protocol over a
local socket (`config/backend.yaml`), a server that never takes a live backend's socket, and a thin client that attaches
with the full snapshot in well under a second and reattaches by itself after link loss or a backend restart.
`boat_tui` does not use it yet.

Keys live in `config/keymap.yaml`: global, tab, panel and view scopes, plus one `popup.<name>` scope per
popup and modal loop (`~/.config/mhseals/keymap.yaml` rebinds any of them). Code asks
`keymap.matches(scope, action, key)` or `keymap.dispatch(scope, key)`; no ui/ file compares a key with a
literal (`test_no_literal_key_checks`). Two keys are `fixed:` on purpose and cannot be rebound: Space stops
in every screen and modal, and E revokes the agent from anywhere with no confirm.
