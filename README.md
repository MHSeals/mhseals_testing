# mhseals_testing

### Overview

This package is the ASV test suite. It checks sensors and TF, drives thruster trials, characterizes the boat, and scores navigation and mission runs. It works against the real boat or the [`crane_ml`](https://github.com/1unarzDev/crane_ml) simulator.

It builds on the other two packages:
- [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware) provides pin-level thruster output, the mixer, the calibrated velocity controller and `thruster_identify`.
- [`mhseals_nav`](https://github.com/mhseals/mhseals_nav) provides localization, Nav2 and the task trees.

The suite never modifies either package's behaviour. It only drives their public topics and launch files.

> Status: every procedure runs against crane_ml end to end; boat runs follow the same paths.

### Usage

```bash
ros2 run mhseals_testing boat_test                                # attach to the boat's running stack
ros2 run mhseals_testing boat_test --start-stack                  # also start localization (FCU auto-detected)
ros2 run mhseals_testing boat_test --profile crane_full_stack --start-stack
ros2 run mhseals_testing thrust_characterize --bag <session_bag> --config <hardware.yaml>
```

`boat_test` starts passive: it shows sensors, TF and advisories, and never commands anything until the outputs are armed and a procedure runs. It is one page of five panels, after [cx](https://github.com/1unarzDev): Status (sensors, TF frames, system health), Motion, Thrusters, Procedures and Activity (results or the event log). The header shows health at a glance; the footer shows the focused panel and its keys. Prompts and confirmations are popups on the same page.

| Key | Action |
| --- | --- |
| Ctrl h j k l, Ctrl arrows | Focus the panel to the left, below, above or right |
| Tab, Shift Tab | Cycle focus through the panels |
| ↑↓, j k | Move within the focused panel; Enter runs or shows details |
| a | Arm panel: outputs, driver, config and every interlock, then arm (typed `ARM` for real pins, a button in simulation) or disarm. The header badge always shows `ARMED` or `DISARMED` |
| Space | Stop: neutral and disarm |
| m | Manual control on the page: hold W/S or ↑↓ (surge), A/D (sway), ←/→ (yaw); +/- speed; Space stops; x or Esc leaves. Terminals with the Kitty keyboard protocol drive held keys and chords; others fall back to tap-repeat |
| +/- | Trial amplitude, in the profile's units |
| x | Write the report |
| ? | Help |
| q | Quit: a popup lists what happens (neutral, disarm, stop children, report, flush the bag) and offers to save an unsaved mapping; the steps then show progress and a summary prints. Esc only backs out, never quits. Ctrl+C runs the same close without the question |
| Thrusters ←→ | Test pulse ±10 µs on the selected thruster, through the armed driver |
| Thrusters 0 | All test pulses back to neutral (Tab away also ends them) |
| Thrusters Enter, ↑↓ | Pick up a thruster and move it; it swaps outputs with each row it passes |
| Thrusters 1-4, r | Wire the thruster to output 1-4 (swapping), or reverse it |
| Thrusters s, u | Save the mapping draft to the config, or undo it |
| Thrusters i | Thruster set: pulse each output, pick the thruster and its direction |
| Activity v | Switch between results and the event log |

The Motion panel's **Drift** line checks the characterization against the water. The first row is the boat's body-frame velocity, live from odometry, as an arrow (bow up: `↑` forward, `←` port, `↺`/`↻` yaw), a speed and a bearing from the bow (`61° port`); while nothing is commanded that velocity is the drift. `fit` is the drift the characterization's bias predicts, and the icon says how far apart they are: `●` within 0.05 m/s, yellow within 0.15, `×` beyond, a sign that current or wind changed since the fit, or that the fit is off. While driving it reads `Vel` and is not compared; right after motion it includes coasting.

Mapping changes stay a draft (marked `unsaved`, changed outputs in yellow; test pulses already use it) until `s` writes them to the hardware YAML (`--config`, default `~/.config/mhseals/hardware.yaml`). Any change to that file, from the panel or another editor, reloads within a second: the suite reloads it and the running `thruster_pwm_node` hot-reloads its copy, no re-arm needed. A bad edit is refused and the last good config stays in use.

Any interlock, child-process exit or error during motion stops at neutral and disarms. Children (the driver, the controller, the bag recorder) get SIGINT if the suite itself dies, even from `kill -9`, so nothing is left driving or recording.

### Procedures

| Key | Procedure | Needs |
| --- | --- | --- |
| 1 2 3 | Surge, sway, yaw trial: drift-corrected response sign, both directions | armed, sensors |
| c | Combined trials: diagonal and turning commands | armed, sensors |
| k | Characterize: fit the velocity model and measured limits, write `<session>.nav2.yaml`; saves the fit on request | armed, sensors, effort commands |
| v | Controller steps: rise, overshoot, settling, steady-state error, IAE | armed, sensors, velocity commands |
| m | Manual control (`keyboard_control`) under the interlocks | armed |
| i | Thruster set: pulse each output, pick the thruster and whether it is reversed; saves on request | armed, thruster outputs |

| n | Nav2 bring-up: lifecycle, costmap and a plan, without moving | Nav2 running |
| w | Waypoint square: cross-track error, closest approach, time | armed, sensors, velocity commands, Nav2 |
| s | Station keeping: RMS radius, drift velocity, time inside the tolerance | armed, sensors, velocity commands, Nav2 |
| o | Perception: camera, LiDAR, detection and track rates; costmap obstacles | |
| t | Mission rehearsal: run a `mhseals_nav` task tree and score it against `config/missions/` | armed, sensors, velocity commands, Nav2 |

Trial timing: `--amplitude`, `--baseline`, `--command-duration`, `--settle`, `--repeats`. Navigation: `--nav-distance`, `--station-hold`, `--station-tolerance`, `--observe`.

Open-loop procedures (trials, characterization, controller steps, manual) refuse to run while another node publishes the command topic, so stop Nav2 first; Nav2 procedures only supervise and need it running. A typical first session on a new boat or simulator: characterize with effort commands, save, then restart with `--velocity-control` and Nav2 for the navigation and mission procedures.

Every session records one rosbag (`bags/<profile>_<time>` by default, `--output` to change). Procedure start and stop events are published on `/mhseals_testing/events` inside it, so any result can be re-analysed from the bag. The JSON report sits next to the bag, and child-process logs go to `bags/logs/<session>/`.

### Profiles

A profile describes the plant the suite drives. Procedures read the profile instead of branching on "boat or simulator". Shipped profiles live in `config/profiles/`; `--profile <path.yaml>` accepts your own.

| Profile | Command | Odometry | Thruster outputs | Bring-up |
| --- | --- | --- | --- | --- |
| `boat` | `/cmd_vel`, effort | `/odom/local` (EKF) | ODROID PWM pins | localization with MAVROS (`fcu:=true`) |
| `crane_full_stack` | `/cmd_vel`, effort | `/odom/local` (EKF), truth `/crane/odom` | `/thrusters/pwm` to crane_ml | localization without an FCU (`fcu:=false`), sim time |
| `crane_direct` | `/crane/cmd_vel_stamped`, m/s | `/crane/odom` | none | attach only, for quick UI work |

`--velocity-control` puts `calibrated_velocity_controller` in front of the thruster outputs, so commands are in m/s and rad/s instead of normalized effort.

Start crane_ml for `crane_full_stack` with `--crane-disable-sitl --crane-ros-thruster-pwm --crane-roboboat-contract`. No ArduPilot or MAVROS process runs in simulation.

### Readiness

Only sensors and TF block motion:
- every `required` sensor of the profile must be live, with valid data and fresh timestamps;
- every required TF chain (`map → odom`, `odom → base_link`) must be fresh, acyclic and have a single publisher.

Power and safety, clock and links, and compute health are advisories. They are always shown on the dashboard's health view and never block, because the operator owns the e-stop. A bench override skips the blocking checks for thruster tests with no sensors; procedures that score motion still refuse to run then.

### Architecture

```
mhseals_testing/
  profiles.py validation.py monitor.py readiness.py report.py   # pure core, no ROS
  analysis/      # trial, characterization (more: step response, nav metrics, mission)
  procedures/    # one run(ctx) per procedure
  ros/           # the only code that imports rclpy
  ui/            # Rich dashboard and the interactive app
config/profiles/  docs/math.tex  launch/
```

Everything outside `ros/` imports without ROS; `test/test_import_boundaries.py` enforces this. The math behind each analysis goes in `docs/math.tex`, built to a PDF in CI.

### Development

```bash
python -m pip install pytest rich PyYAML numpy
PYTHONPATH=.:../mhseals_hardware python -m pytest -q test
```

CI runs the same tests on Python 3.10, 3.12 and 3.13 without ROS.
