# mhseals_testing

### Overview

This package is the ASV test suite. It checks sensors and TF, drives thruster trials, characterizes the boat, and scores navigation and mission runs. It works against the real boat or the [`crane_ml`](https://github.com/1unarzDev/crane_ml) simulator.

It builds on the other two packages:
- [`mhseals_hardware`](https://github.com/mhseals/mhseals_hardware) provides pin-level thruster output, the mixer, the calibrated velocity controller and `thruster_identify`.
- [`mhseals_nav`](https://github.com/mhseals/mhseals_nav) provides localization, Nav2 and the task trees.

The suite never modifies either package's behaviour. It only drives their public topics and launch files.

> Status: the pure core (profiles, validation, monitor, readiness, trial and characterization analysis, dashboard, report) is in place. The interactive `boat_test` TUI, ROS adapters and procedures are being added next.

### Usage

```bash
ros2 run mhseals_testing thrust_characterize --bag <session_bag> --config <hardware.yaml>
ros2 run mhseals_testing thrust_characterize --samples samples.json --nav2-params nav2.yaml --nav2-overlay out.yaml
```

Every session records one rosbag. Procedure start and stop events are published inside it, so any result can be re-analysed from the bag. A JSON report is written next to the bag.

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
