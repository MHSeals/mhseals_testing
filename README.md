# mhseals_testing

### Overview

The ASV test suite: sensor and TF checks, thruster trials, characterization, controller, navigation and perception tests, and scored mission rehearsals, on the boat or the [`crane_ml`](https://github.com/1unarzDev/crane_ml) simulator. It drives [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware) (thruster driver, velocity controller) and [`mhseals_nav`](https://github.com/mhseals/mhseals_nav) (localization, Nav2, task trees) through their public topics and launch files only.

### Usage

```bash
ros2 run mhseals_testing boat_test                                   # attach to a running stack
ros2 run mhseals_testing boat_test --start-stack                     # also start localization
ros2 run mhseals_testing boat_test --profile crane_full_stack --start-stack
ros2 run mhseals_testing thrust_characterize --bag <session_bag> --config <hardware.yaml>
```

`boat_test` starts passive and commands nothing until the outputs are armed. It is one page of five panels: Status (sensors, TF, system health), Motion, Thrusters, Procedures and Activity (results or log). The header shows health and `● Armed`/`● Disarmed`; the footer shows the keys for the focused panel; prompts are popups. `?` lists every key.

| Key | Action |
| --- | --- |
| Ctrl hjkl / Ctrl arrows, Tab | Focus a neighbouring panel / cycle panels |
| ↑↓ j, Enter | Move in a panel; run or show details |
| a | Arm panel: outputs, driver, config and interlocks; arm (typed `ARM` on real pins) or disarm |
| Space | Stop: neutral and disarm |
| m | Manual control: hold W/S, A/D and ←/→ together; +/- speed; x leaves |
| q, Ctrl+C | Quit: neutral, disarm, stop children, write the report, flush the bag |
| Thrusters ←→, 0 | Test pulse ±10 µs on the selected thruster; back to neutral |
| Thrusters Enter, 1-4, r | Move a thruster, wire it to an output, reverse it (a draft) |
| Thrusters s, u, i, c | Save or undo the draft; thruster set (identify by pulses); clear the decoupling |

Held-key chords need a terminal with the Kitty keyboard protocol (Kitty, Ghostty, foot, WezTerm); others fall back to tap-repeat. Saved mappings hot-reload into the running driver. The Motion panel's drift line shows the live body-frame velocity (bow up, with a bearing) against what the model predicts: the boat's mechanical bias plus the current, turned into the body by the heading (the velocity controller's live estimate, else the characterization's).

### Procedures

| Key | Procedure | Needs |
| --- | --- | --- |
| 1 2 3, c | Surge, sway, yaw and combined trials: drift-corrected response | armed, sensors |
| k | Characterize: gain, mechanical bias, decoupling, today's current, limits, Nav2 limits | armed, sensors, effort |
| v | Controller steps: rise, overshoot, settling, error, IAE | armed, sensors, velocity |
| m, i | Manual control; thruster set | armed |
| n, o | Nav2 bring-up; perception rates and costmap | — |
| w, s | Waypoint square; station keeping | armed, sensors, velocity, Nav2 |
| t | Mission rehearsal: run an `mhseals_nav` task (`pair_test`, `circle_test`, a 2025 task or the whole `mission`), score its events (`config/missions/`) and, in crane_ml, have the referee judge the true track against the course | armed, sensors, velocity, Nav2 |

Procedure shortcuts work from every panel; focused-panel wiring keys and Activity’s `v` keep their local meaning (use Procedures/Enter for those tests). `k` always opens Characterize; use ↑ to move up. Choosing a procedure that needs arming opens the arm panel first. Open-loop procedures refuse to run while another node publishes the command topic, so characterize with Nav2 stopped, then restart with `--velocity-control` and Nav2 for the navigation procedures. Timing: `--amplitude`, `--baseline`, `--command-duration`, `--settle`, `--repeats`, `--nav-distance`, `--station-hold`, `--station-tolerance`, `--observe`.

Only sensors and TF block motion (required sensors live and fresh; `map → odom → base_link` fresh, acyclic, single publisher). Power, clock, links and compute are advisories. `--bench-override` skips the blocking checks for bench tests; procedures that score motion still refuse.

Characterization saves only explicit hardware overrides. Motion limits are the speeds and accelerations observed at the tested effort; partial-throttle runs do not grant unmeasured full-throttle limits. Review them before use. The generated `<session>.nav2.yaml` contains only limits; restart Nav2 with `ros2 launch mhseals_nav navigation.launch.py sim:=false limits_file:=<session>.nav2.yaml`. Current planner/costmap settings and the reverse-speed cap remain in the base profile. Regenerate older full-profile overlays. Offline: add `--nav2-params <nav2_params.yaml> --nav2-overlay <limits.yaml>` to `thrust_characterize`.

### Profiles

| Profile | Command | Odometry | Outputs | Bring-up |
| --- | --- | --- | --- | --- |
| `boat` | `/cmd_vel`, effort | `/odom/local` | ODROID PWM pins | MAVROS + EKFs |
| `crane_full_stack` | `/cmd_vel`, effort | `/odom/local`, truth `/crane/odom` | `/thrusters/pwm` | EKFs without an FCU, sim time |
| `crane_direct` | `/crane/cmd_vel_stamped`, m/s | `/crane/odom` | none | attach only |

`--velocity-control` puts `calibrated_velocity_controller` in front of the outputs (commands in m/s). `--profile <file.yaml>` loads your own. Start crane_ml for `crane_full_stack` with `--crane-disable-sitl --crane-ros-thruster-pwm --crane-roboboat-contract --crane-roboboat-course Tools/RoboBoat/courses/roboboat_2025.json --crane-roboboat-course-truth <bags>/course_truth.json --crane-lidar-returns-only --crane-lidar-water-level 0.02`, adding `--crane-wind 8,90 --crane-water-current 0.15,0 --crane-disturbance-after 60` for a moderate breeze and current. The referee reads `course_truth` from the profile.

For `pair_test` or `circle_test`, use this package's `config/courses/<task>.json` as the simulator course and choose the matching task under `t`. Negate both buoy `y` coordinates in the pair course to swap the colours. These courses tag their objects for the referee; without matching course truth it reports `N/A`, while event scoring still runs. Task distances and timeouts live in `mhseals_nav/config/tasks/`.

### Data

Each session records one bag (`bags/<profile>_<time>`, `--output`), with procedure events on `/mhseals_testing/events`, plus `<session>.report.json`, the driver's `<session>.hardware.yaml` and logs in `bags/logs/<session>/`. Children stop with the suite, even if it is killed.

### Development

```bash
python -m pip install pytest rich PyYAML numpy
PYTHONPATH=.:../mhseals_hardware python -m pytest -q test
make -C docs docker-pdf   # docs/math.pdf: the math behind every analysis
```

Everything outside `mhseals_testing/ros/` imports without ROS (enforced by `test/test_import_boundaries.py`). CI runs the tests on Python 3.10, 3.12 and 3.13 and builds `math.pdf`.
