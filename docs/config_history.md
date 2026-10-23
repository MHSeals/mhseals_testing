# Config history

Every change to a config subsystem (`hardware`, `nav2`, `fcu`, `rtk`, `health`, `keymap`) is one line in an
append-only log under `$MHSEALS_DATA/config-history/`:

```
changes.jsonl                      # one change or label per line
versions/<subsystem>/<sha12>.json  # content-addressed snapshots, written once
lock                               # flock for appends and the id counter
```

A change has an id (`c0001`, ...), the `before` and `after` snapshot shas, a `source` and `applied`.
Sources: a writer such as `tui:pins`, `external` (found by `observe`, `applied: observed`), `seed` or `import`.
Names and notes are label lines appended later, so the log is never rewritten. The newest label wins.
Dedupe is by content: recording the head again does nothing.

## Seeding (first run)

`mhseals_testing.history.seed.seed(history, readers, fcu_param=..., hardware_snapshots=...)` does three things.
Running it again records nothing.

1. **Hardware snapshots (optional).** Each `*.hardware.yaml` in the given folder that is not already a stored
   version becomes an `import` change, oldest file time first, dated with the file time. There are no trial links:
   the old report format cannot be linked without guessing.
2. **FCU baseline.** If the FCU has no history, the `.param` file (`NAME,value` lines, normally
   `mhseals_nav/config/fcu/blastoise.param`) becomes its first version, named
   "audit 10-22 (GSF yaw, no compass)". The note records the 10-19 change (`EK3_SRC1_YAW` 1 to 8,
   `COMPASS_USE` 1 to 0). Counters and gyro offsets (`STAT_*`, `INS_GYROFFS_*`, ...) are not stored.
3. **Observe.** Each reader returns `(value, raw, meta)` or None; the value found becomes the head, so a fresh
   store starts at `c0001`. Imports run before this step, so the current state is always the head.

The design is in `config_history.md` in the plan folder (CH10). The `boat_trial config seed` verb that calls this
arrives with the config CLI (ts088).
