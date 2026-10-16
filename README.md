# mhseals_testing

### Overview

This package is the ASV test suite. It checks sensors and TF, drives thruster trials, characterizes the boat, and scores navigation and mission runs. It works against the real boat or the [`crane_ml`](https://github.com/1unarzDev/crane_ml) simulator.

It builds on the other two packages:
- [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware) provides pin-level thruster output, the mixer, the calibrated velocity controller and `thruster_identify`.
- [`mhseals_nav`](https://github.com/mhseals/mhseals_nav) provides localization, Nav2 and the task trees.

The suite never modifies either package's behaviour. It only drives their public topics and launch files.

> Status: monitoring, readiness, advisories, bags, reports, arming and the actuation procedures work on the boat and in crane_ml. Navigation, perception and mission procedures are next.

### Usage

```bash
ros2 run mhseals_testing boat_test                                # attach to the boat's running stack
ros2 run mhseals_testing boat_test --start-stack                  # also start localization (FCU auto-detected)
ros2 run mhseals_testing boat_test --profile crane_full_stack --start-stack
ros2 run mhseals_testing thrust_characterize --bag <session_bag> --config <hardware.yaml>
```

`boat_test` starts passive: it shows sensors, TF and advisories, and never commands anything until the outputs are armed and a procedure runs.

| Key | Action |
| --- | --- |
| Tab, ←/→ | Cycle views: overview, sensors, TF, health, mapping, results, logs |
| H | Arm (type `ARM`) or disarm. Arming starts `thruster_pwm_node`, plus `calibrated_velocity_controller` with `--velocity-control` |
| P | Choose a procedure; unavailable ones say why |
| M | Manual control |
| +/- | Trial amplitude, in the profile's units |
| Space | Stop: neutral and disarm |
| E | Write the report |
| Q | Quit (neutral, disarm, flush the bag) |

Any interlock, child-process exit or error during motion stops at neutral and disarms.

### Procedures

| Key | Procedure | Needs |
| --- | --- | --- |
| 1 2 3 | Surge, sway, yaw trial: drift-corrected response sign, both directions | armed, sensors |
| c | Combined trials: diagonal and turning commands | armed, sensors |
| k | Characterize: fit the velocity model and measured limits, write `<session>.nav2.yaml`; saves the fit on request | armed, sensors, effort commands |
| v | Controller steps: rise, overshoot, settling, steady-state error, IAE | armed, sensors, velocity commands |
| m | Manual control (`keyboard_control`) under the interlocks | armed |
| i | Thruster mapping (`thruster_identify`, pulses or by hand); disarms first | thruster outputs |

Trial timing: `--amplitude`, `--baseline`, `--command-duration`, `--settle`, `--repeats`.

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
