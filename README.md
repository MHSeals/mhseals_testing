# mhseals_testing

### Overview

The ASV test suite: sensor and TF checks, thruster trials, characterization, controller, navigation and perception tests, and scored mission rehearsals, on the boat or the [`crane_ml`](https://github.com/1unarzDev/crane_ml) simulator. It drives [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware) (thruster driver, velocity controller) and [`mhseals_nav`](https://github.com/mhseals/mhseals_nav) (localization, Nav2, task trees) through their public topics and launch files only.

### Usage

```bash
ros2 run mhseals_testing boat_test                                   # attach to a running stack
ros2 run mhseals_testing boat_test --start-stack                     # also start the profile's odometry (more from the Status rows)
ros2 run mhseals_testing boat_test --profile crane_full_stack --start-stack
```

`boat_test` starts passive and commands nothing until the outputs are armed. It is one page of five panels: Status (sensors, TF, system health), Motion, Thrusters, Procedures and Activity (results or log). The header shows health and `✓ Disarmed` / `! Armed`; the footer shows the keys for the focused panel and always keeps Space Stop, Help and Quit; prompts are popups that scroll when the terminal is short. `?` lists every key. Letter keys ignore Shift and Caps Lock. Below 60 columns the Status column folds away; below 36×10 the page becomes an enlarge-terminal hint that still shows the armed state and Space stop.

Every state has its own glyph (✓ ok, ! warn, × fail, ◌ busy, ○ off), so the page reads without color (`NO_COLOR`). `TERM=dumb` or `MHS_ASCII=1` draws pure ASCII; `MHS_ASCII=0` leaves it off.

`?` lists every key and is the reference; the ones to know are:

- **a** arm panel (outputs, driver, command mux, config, interlocks; typed `ARM` on real pins), **Space** stop (neutral and disarm), **q** quit (neutral, disarm, stop children, write the report, flush the bag);
- **m** manual control (hold W/S, A/D and ←/→ together; +/- speed; x leaves);
- Thrusters: ←→ and 0 test-pulse ±10 µs / back to neutral, Enter, 1-4, r edit a draft mapping, s/u save or undo it, i thruster set, c clear the decoupling.

Held-key chords need a terminal with the Kitty keyboard protocol (Kitty, Ghostty, foot, WezTerm); others fall back to tap-repeat. Saved mappings hot-reload into the running driver. The Motion panel's drift line shows the live body-frame velocity (bow up, with a bearing) against what the model predicts: the boat's mechanical bias plus the current, turned into the body by the heading (the velocity controller's live estimate, else the characterization's).

Status checks **PWM driver** (managed process and ROS subscription) and **PWM readback** (enabled outputs, pulse widths and frequency, once per second); Enter shows details. Run on the ODROID for pin readback. Disabled pins are normal before arming; missing/disabled pins while armed and unexpected drivers/outputs are flagged. An enabled 1500 µs output still pulses at neutral. Readback is the kernel configuration, not measured voltage, MOSFET/ESC power or proof of thruster movement.

### Procedures

The menu groups them: Calibrate (f, g, v), Drive (m, i), Navigate (n, w, s), Mission (t), Perception (p).

| Key | Procedure | Needs |
| --- | --- | --- |
| f | Characterize: speed curves per axis and sign, mechanical bias, today's current, limits, Nav2 limits | armed, sensors, effort |
| v | Controller steps: rise, overshoot, settling, error, IAE | armed, sensors, velocity |
| g, e | Compass calibration from a handheld bearing (42006, written only on your yes); compass interference against thruster effort | —; armed, outputs |
| m, i | Manual control; thruster set | armed |
| n | Nav2 bring-up | — |
| p | Perception rates and costmap | — |
| w, s | Waypoint square; station keeping | armed, sensors, velocity, Nav2 |
| t | Mission rehearsal: run an `mhseals_nav` task (`pair_test`, `circle_test`, a 2025 task or the whole `mission`), score its events (`config/missions/`) and, in crane_ml, have the referee judge the true track against the course | armed, sensors, velocity, Nav2 |

Procedure shortcuts work from every panel; focused-panel wiring keys and Activity’s `v` keep their local meaning (use Procedures/Enter for those tests). `f` opens Characterize (fit); `j`/`k` move down/up, including in popups. Choosing a procedure that needs arming opens the arm panel first. Open-loop procedures refuse to run while another node publishes the command topic, so characterize with Nav2 stopped, then restart with `--set command_mode=velocity` and Nav2 for the navigation procedures. Timing and size are settings (`harness/settings.py`; last used values persist in `~/.config/mhseals/boat_test.yaml`, except command mode and bench override): `--set amplitude=0.4`, `baseline`, `command`, `settle`, `repeats`, `nav_distance`, `station_hold`, `station_tolerance`, `observe`, `turn`, `stale_after`.

Only sensors and TF block motion (required sensors live and fresh; `map → odom → base_link` fresh, acyclic, single publisher). Power, clock, links and compute are advisories. `--set bench_override=on` skips the blocking checks for bench tests; procedures that score motion still refuse.

Use `i` to verify wiring and reversals before running the trials.

Characterization stages an identity correction, surveys every direction, refines the correction columns C+/C- per direction (`--direction sway+` reruns one) and saves speed curves and C+/C- only on request; an abort restores the saved correction. Motion limits are the speeds and accelerations observed at the tested effort; partial-throttle runs do not grant unmeasured full-throttle limits. Review them before use. The generated `<session>.nav2.yaml` contains only limits; restart Nav2 with `ros2 launch mhseals_nav navigation.launch.py sim:=false limits_file:=<session>.nav2.yaml`. Current planner/costmap settings and the reverse-speed cap remain in the base profile. Regenerate older full-profile overlays.

### Profiles

| Profile | Command | Odometry | Outputs | Bring-up |
| --- | --- | --- | --- | --- |
| `boat` | `/cmd_vel`, effort | `/odom/local` | ODROID PWM pins | MAVROS + EKFs |
| `crane_full_stack` | `/cmd_vel`, effort | `/odom/local`, truth `/crane/odom` | `/thrusters/pwm` | EKFs without an FCU, sim time |
| `crane_direct` | `/crane/cmd_vel_stamped`, m/s | `/crane/odom` | none | attach only |

`--set command_mode=velocity` puts `calibrated_velocity_controller` in front of the outputs (commands in m/s). On profiles that command `/cmd_vel` (`boat`, `crane_full_stack`), arming also starts `mhseals_hardware`'s `command_mux`, which arbitrates RC, TUI keys, holds and autonomy onto `/cmd_vel/applied`; the driver, or the controller in front of it, listens there. `--profile <file.yaml>` loads your own. Start crane_ml for `crane_full_stack` with `--crane-disable-sitl --crane-ros-thruster-pwm --crane-roboboat-contract --crane-roboboat-course Tools/RoboBoat/courses/roboboat_2025.json --crane-roboboat-course-truth <bags>/course_truth.json --crane-lidar-returns-only --crane-lidar-water-level 0.02`, adding `--crane-wind 8,90 --crane-water-current 0.15,0 --crane-disturbance-after 60` for a moderate breeze and current. The referee reads `course_truth` from the profile.

For `pair_test` or `circle_test`, use this package's `config/courses/<task>.json` as the simulator course and choose the matching task under `t`. Negate both buoy `y` coordinates in the pair course to swap the colours. These courses tag their objects for the referee; without matching course truth it reports `N/A`, while event scoring still runs. Task distances and timeouts live in `mhseals_nav/config/tasks/`.

### Data

Records go under one data root: `$MHSEALS_DATA`, else `$ROS_WS/bags`, else `~/astro_dock/bags`, never the working directory ([docs/data.md](docs/data.md)). Each session records one bag (`<root>/<profile>_<time>`, `--output`) of every topic except images, point clouds and costmaps (`config/bags.yaml`), with procedure events latched on `/mhseals_testing/events`, plus `<session>.report.json`, the driver's `<session>.hardware.yaml` and logs in `<root>/logs/<session>/`. Children stop with the suite, even if it is killed. Sessions are the flat layout in [docs/data.md](docs/data.md); the report and bag name are the session identity.

### Health

Health checks are YAML (`config/health/*.yaml`) over facts that probes publish; a `fail` blocks a procedure from starting, a `warn` shows its reason, and nothing interrupts a run. `~/.config/mhseals/health.yaml` can `disable` or `override` checks. How to add a fact, a kind or a check is in [docs/health.md](docs/health.md).

### Development

```bash
python -m pip install pytest rich PyYAML numpy
PYTHONPATH=.:../mhseals_hardware python -m pytest -q test
make -C docs docker-pdf   # docs/math.pdf: the math behind every analysis
```

`docs/data.md` describes the data layout; `docs/health.md` the health system; [docs/field-2026-10-18.md](docs/field-2026-10-18.md) holds the field-day findings (thrust coupling, method lessons, next holds) that shaped the speed curves and per-sign decoupling; the raw day notes are not kept in the repo.

Everything outside `mhseals_testing/ros/` imports without ROS (enforced by `test/test_import_boundaries.py`). CI runs the tests on Python 3.10, 3.12 and 3.13 and builds `math.pdf`.
