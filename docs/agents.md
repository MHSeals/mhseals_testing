# Agent control

Reference for `tools/boat_agent` and the agent hook; the short version is in the [README](../README.md#agents).
The playbook an agent gets is `python3 .agents/hook.py --playbook`.

- Agents drive the boat only through the owner TUI: `boat_trial claim | status | follow | run | goal | start | stop |
  arm | reload | set | fcu | mark | note | ask | release` (remotely `ssh blastoise-odroid
  ~/astro_dock/src/mhseals_testing/tools/boat_agent <verb> --who NAME`) runs each request through the same gates, arm
  panel and runner as a key and prints JSON (exit 0 ok, 2 refused with a `hint`, 3 revoked or overridden, 4 no TUI, 5
  blocked, 6 timeout, 7 paused).
- The operator's `S` declares manual, battery swap or power cycle (`m` manual control sets manual): agents' changing
  verbs exit 7 with since and ETA, claim leases suspend (no lease hold), `status` carries `operator` and, after a swap
  or power cycle, `restarted_after`; with `BOAT_AGENT_TARGET` set, `tools/boat_agent` makes the ssh hop itself and an
  unreachable boat answers `unreachable` (exit 4) with the last operator state it saw.
- `arm --claim ID` arms through the arm panel's path after a countdown banner (`config/agent.yaml` `arm.countdown`, 5
  s, which an agent may lengthen but never shorten) the operator cancels with Esc or Enter; a latched e-stop refuses
  it, and the ARMED chip and journal name the agent.
- A `run`, `goal` or `ask` that finds the TUI busy (a popup open, a procedure or another agent act running, the
  operator driving manually) is answered `queued` with the cause and starts when that ends, through every gate as if
  just sent, for up to `config/agent.yaml` `queue_s` (120 s), then `busy_ui` (exit 2); a queued run or goal whose CLI
  stops renewing it (killed while it waited) is answered `expired` and never starts. `arm` is refused at once.
- `reload config [NAME]` re-reads the TUI's watched YAML as a hand edit does; `reload COMPONENT --claim ID` stops and
  starts one part under the `start`/`stop` rules.
- Characterize (`f`, `r`/`R` on Correction, `run characterize`) stages identity C on the running driver over
  SetParameters and reads it back, puts the deployed C back while a human drives a pause, and shows a progress bar
  with the command against measured velocity.
- `run KEY --sim-arm` arms only a stack every check of `harness/sim_arm.py` proves a simulator (sim profile and
  `/clock`, the crane player, a private domain never 42, localhost discovery, a private-IPC container, no MAVROS or
  other hardware node, the session's empty hold file, no `<data root>/agent/stop`), re-checks the graph every 5 s and
  disarms when the run ends or the re-check fails; it is off as shipped (`config/agent.yaml` is the boat's too) and
  the sim container turns it on with `docker run -e MHSEALS_SIM_ARM=1`.
- A brand-new sim session clears holds a previous session left in its own hold file (logged), and a `--sim-arm` run's
  Ready prompts are confirmed by the agent; real-boat and other runs keep the human's.
- The hook blocks starting the driver or mux by any route and changing the mux's saved holds or the stop file.
- `start COMPONENT --claim ID` and `stop COMPONENT --claim ID` start and stop a stack part that `config/agent.yaml`
  `start:` lists (the odometry stack, EKFs, sensors, camera and objects) through its Status row's own start path,
  without the popup; the row reads `by agent W` until it stops, and the notice and journal name the agent.
- They never start a commander part (Nav2, the mission runner), the driver, mux, controller, heartbeat or watchdogs,
  refuse a stop while armed or one that would stop a part not on the list, and on a boat the hook routes a
  hand-started device launch or `mhseals-*` unit to them.
- `fcu get PARAM...` and `fcu rates` (IMU and odometry rates, gap p99, FCU load, "Main loop slow") read only; `fcu set
  PARAM VALUE... --claim ID`, `fcu revert --to SNAPSHOT --claim ID` and `fcu restart --claim ID` (disarmed) go through
  the TUI's MAVROS link and write only what `config/agent.yaml` `fcu.set` lists, never the yaw source, compass,
  servos, RC, arming or failsafes; the first write snapshots every parameter (its sha256 in the ledger), the FCU is
  re-checked disarmed before each write, and each is read back and logged to `fcu_changes.md` with the agent's id as
  it lands, so a partial set is on record; revert takes only a store snapshot whose checksum matches and checks every
  value like a set, and SCHED_LOOP_RATE 400 needs a rates check this TUI ran after a restart that followed the last
  loop-rate write.
- Space, RC and `E` always win.
- Leases, allowed procedures and the agent hook are in `config/agent.yaml`. astro_dock's `.agents/hook.py` runs this
  package's hook, `mhseals_testing/agent_hook.py`, for Claude Code and Codex.
- It gives the agent this playbook, blocks raw motion and FCU commands with the verb to use instead, and shows the
  agent's boat commands in the TUI: this machine's TUI, unless `BOAT_AGENT_TARGET=blastoise-odroid` in the agent's
  environment names a boat session's host (only then does the hook ssh there), so sim and dev work never touches the
  boat (`blastoise` still works for one release; it means blastoise-odroid).
- `boat_trial analyze referee <session>`, `index` (`--json`) and `check --require GROUP` re-judge, index and gate
  finished sessions.
- `run KEY --set K=V` (applied before the run starts, named in the Allow popup, refused while a procedure runs) and
  `set K=V --claim ID` (disarmed and idle) change a setting that `config/agent.yaml` `settings:` lists for that
  procedure (`thrust_step`, `dock_step`, `auto_tab`), within its choices, as the operator's change does; the notice, a
  toast and the journal say `agent W set K=V`, and amplitude, timings, repeats, navigation extents, the command mode,
  the bench override, staleness and the agent controls are never an agent's, listed or not.
- `release HOLD --why "..."` frees a lease or watchdog hold once its fault cleared; an operator hold (Space,
  stop, manual) only with a grant in `~/.config/mhseals/agent_grants.yaml`; the e-stop never. Disarmed, the mux
  is stopped: the release then judges and clears the holds it saved for the next arm, by the same rules.
- The owner's preview (`boat_tui preview`) is theirs: the hook denies starting, stopping or typing into it. An
  integration test runs its own: `boat_tui preview --headless --test`, then `boat_tui preview down --test`.
