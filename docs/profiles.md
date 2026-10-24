# Profiles and simulator courses

| Profile | Command | Odometry | Outputs | Bring-up |
| --- | --- | --- | --- | --- |
| `boat` (real boat) | `/cmd_vel`, effort | `/odom/local`, physical FCU/sensors | Real ODROID PWM pins | MAVROS + EKFs; navigation available |
| `crane_full_stack` (Unity through navigation) | `/cmd_vel`, effort | `/odom/local`, simulated sensors; truth `/crane/odom` | ROS `/thrusters/pwm` into Unity | EKFs without an FCU; navigation available, sim time |
| `crane_direct` (Unity direct) | `/crane/cmd_vel_stamped`, m/s | Unity ground truth `/crane/odom` | No PWM driver | Attach directly; bypasses mhseals_nav |

The profile IDs stay unchanged. `boat_trial profiles` reads their descriptions from the
profile YAML; TUI `?` begins with the active profile description. Direct Unity is for
testing commands against Unity's own velocity controller/truth; full-stack Unity
exercises the boat's ROS PWM and sensor/navigation path. Components without `start:
true` still start from Status rows, rather than automatically when selecting a profile.

`--set command_mode=velocity` puts `calibrated_velocity_controller` in front of the
outputs (commands in m/s). On profiles that command `/cmd_vel` (`boat`,
`crane_full_stack`), arming also starts `mhseals_hardware`'s `command_mux`, which
arbitrates RC, TUI keys, holds and autonomy onto `/cmd_vel/applied`; the driver, or the
controller in front of it, listens there. `--profile <file.yaml>` loads your own. Start
crane_ml for `crane_full_stack` with `--crane-disable-sitl --crane-ros-thruster-pwm
--crane-roboboat-contract --crane-roboboat-course
Tools/RoboBoat/courses/roboboat_2025.json --crane-roboboat-course-truth
<bags>/course_truth.json --crane-lidar-returns-only --crane-lidar-water-level 0.02`,
adding `--crane-wind 8,90 --crane-water-current 0.15,0 --crane-disturbance-after 60` for
a moderate breeze and current. The referee reads `course_truth` from the profile.

For `pair_test` or `circle_test`, this package's `config/courses/<task>.yaml` is the
editable course. Pass `--crane-roboboat-course "$(boat_trial course circle_test)"` (or
`pair_test`) to your existing Unity command and choose the matching task under `t`.
Negate both buoy `y` coordinates in the pair course to swap the colours. These courses
tag their objects for the referee; without matching course truth it reports `N/A`, while
event scoring still runs. Task distances and timeouts live in
`mhseals_nav/config/tasks/`.

`boat_trial course <name-or-YAML-path>` validates the YAML and prints only an absolute
generated JSON path; invalid input returns nonzero on stderr. Bare names resolve
installed package resources outside the working directory; custom `.yaml`/`.yml` paths
select your own file. Every invocation compiles the current YAML into deterministic
content-addressed JSON under `$XDG_CACHE_HOME/mhseals_testing/course-json` (default
`~/.cache`). That disposable artifact is Unity input, not another editable
configuration: edit YAML and run the same command again. The quoted substitution handles
cache paths with spaces. The helper neither launches Unity nor samples the boat.
`course_truth.json`, session JSON and runtime recordings remain data in their existing
formats.


Odometry startup defaults belong to `mhseals_nav/config/odometry_startup.yaml`.
The real boat uses the `included` startup context (the packaged six-second FCU
settling delay); Unity through navigation uses `standalone` (the packaged zero
delay). These contexts preserve the existing sequencing without overriding YAML
values. `crane_direct` continues to bypass the EKFs and this selection.

Set `odometry_startup_file` (operator) and `odometry_deployment_file` (deployment)
in a profile component's launch `args` to select additional owner YAML layers.
The TUI forwards the same selection, context and explicit compatibility aliases
to odometry and the separate EKF child; conflicting selections refuse before a
process starts. Explicit `ekf_delay` remains supported. Fresh sessions consume
new YAML edits through the navigation owner's resolver.

Runtime EKF restart remains unavailable in the application. Its inert command
seam retains explicit selectors, but this is not proof of same-session frozen
odometry inputs, filter-state preservation or safe consumer resumption. Enabling
that lifecycle requires separate owner validation.
