# TUI images inside tmux

`theme.yaml graphics: auto|kitty|sixel` draws images under tmux only when tmux confirms every client
viewing the TUI's window can show them (`ui/pixels.py`). If it can't confirm that, the TUI uses text.

| Path | Needs |
|---|---|
| sixel (foot, WezTerm, xterm -ti 340) | tmux >= 3.4 built with sixel (`tmux -V`; the TUI checks tmux's DA1); every viewing client lists `sixel` in `tmux display -p '#{client_termfeatures}'` and reports a cell size |
| kitty (kitty, Ghostty) | `set -g allow-passthrough on` (or `all`); every viewing client is kitty/Ghostty (`#{client_termtype}`) |

- Detected live, once a second: when a client attaches, leaves or switches to the window, the TUI
  changes tier or repaints. If a viewer can't show images (tmux would draw `SIXEL IMAGE +++` for it),
  everyone gets text until it leaves. Images are also repainted every 10 s, because on a redraw
  (reattach, window switch, popup) tmux keeps only its last 19 sixel images server-wide.
- **Nested tmux stays text.** That covers foot -> cx (tmux) -> ssh -> the boat's `boat` tmux:
  the boat's tmux can't see the terminal behind cx. Images work when the TUI's tmux is the one
  your terminal attaches to directly:
  - innovation: cx is tmux 3.8 with sixel, and foot reports `sixel`, so the TUI in a cx pane shows sixel.
  - boat (tmux 3.7c): images need a terminal attached straight to the boat's tmux
    (`ssh -t blastoise-odroid tmux attach -t boat` from foot, outside cx), and that tmux
    built with sixel (Ubuntu/Debian builds: check `tmux display -p '#{client_termfeatures}'`
    lists `sixel` while attached from foot).
- Over ssh with no tmux on the TUI's host (`$TMUX` unset there) but a tmux on your side, the TUI
  only sees XTVERSION `tmux …`, so it uses text.
