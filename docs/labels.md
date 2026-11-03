# Field labels: operator cheat sheet

Marks are instant and never open anything. Space stays the stop; the e-stop is absolute.

## Before the run (disarmed)

| Key | Does |
| --- | --- |
| `L` | Labels tab: wind, waves and current on top |
| `↑↓` then `←→` | Set each: 0 unset, 1 calm, 2 moderate, 3 rough (change it whenever conditions change) |
| `[` / `]` | Back to Drive |

## During the run (any tab, in a procedure, in manual)

| Key | Does |
| --- | --- |
| `b` | Mark now. Within 3 s a digit tags it (footer): 1 wake, 2 gust, 3 contact, 4 pushed, 5 sensor bump, 6 fouling, 7 good, 8 interesting, 9 other. No digit: an untagged mark |
| `N` | Bad data from here; `N` again ends it (bumped the dock, someone on the boat, sensor unplugged) |
| `"` | You just gave an instruction: logged now with the pose. Within 3 s a digit picks the words (1 hold position here, 2 go to the next gate, ...); other words later in Labels |
| `B` | A note on the last mark (stopped only) |

On the Thrusters panel the digits stay the panel's (0 is neutral); tag those marks in Labels.

Automatic, nothing to press: procedure start and end, characterization steps, arm and disarm, takeovers
(keys or RC over a procedure, agent or goal), link-loss holds, fence exits, GNSS fix changes, and steady
command windows (calibration candidates).

## After the run (disarmed, on the station)

The footer shows `L Review N` while marks wait. In Labels:

| Key | Does |
| --- | --- |
| `↑↓` | Select an item |
| `y` / `r` | Accept / reject (again: back to pending). Rejected marks stop downgrading the data |
| `1-9` | Tag a mark; on a said item, its outcome: 1 done, 2 partly, 3 failed, 4 unsafe, 5 ignored |
| `n` | Note (the words of a said item) |
| `s` / `m` / `u` | Split at a mark or the middle / merge with the next of its kind / undo the last edit |
| `Enter` | Replay a closed session from the item |
| `<` `>` | Previous / next session |

Every edit is saved at once: `manifest.json` beside the session's bag is what the judge, the plant card and
the NLC study read. Edits wait for disarmed; marks and conditions always work. Agents add marks with
`boat_agent mark "note" --tag wake --why "..."`.
