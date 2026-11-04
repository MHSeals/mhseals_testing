# How boat_tui preview and boat_tui sim derive their stack

The preview (`tools/boat_preview`) and the sim (`tools/boat_sim`) run their own isolated stack: their own
worktrees or overlay, container, ROS domain, endpoint port, tmux socket and data directories (config/preview.yaml,
config/sim/stack.yaml). Everything the workspace already knows is read from the workspace, never written down
again, so neither drifts from the dev and boat builds (D-20261104-03; the 11-04 failures were six such copies).

| What | Where it comes from |
| --- | --- |
| The workspace | `src/` of this checkout's main clone (`sim_stack.src_dir`; `BOAT_PREVIEW_SRC` / `BOAT_SIM_SRC`), and the astro_dock around it (its `hw.sh`) |
| Repos | `hw.sh hosts repos --role ROLE...` of `roles:` (config/sim/stack.yaml: `repos.all` plus those roles of the profile's hosts.yaml, the list setup and hosts apply use), plus astro_dock's submodules (`git ls-tree HEAD src/` gitlinks) |
| Commits | a team repo at `origin/<branch>` (hosts.yaml `sources`), a submodule at its pin. `boat_tui preview REF` takes REF in each repo that has it; a pinned repo only when REF descends from its pin (ROS-TCP-Endpoint's ROS 1 `main` never does) |
| Image | `hw.sh image --local` (astro_dock `.devcontainer/dev-image.sh`): the dev image the Compose config prebuild wrote names (`docker compose ... config --images dev`), or its local UID variant only when built on that tag. `up` first runs prebuild's pull-if-newer (`--pull`) |
| Build | `hw.sh build --ws DIR --container NAME` (astro_dock `devices/build.sh`), the build hosts apply runs too: the workspace's colcon_defaults.yaml, /opt/ros alone (a snapshot has no dev.bashrc), stale links pruned, from clean when the container's image or the colcon defaults changed since the last build |
| Player | `sim_stack.first_player` over stack.yaml `player:` (`BOAT_SIM_PLAYER`), for both |
| Heavy guard | `src/mhseals_agents/tools/heavy` when present; without it they run directly |

Every gap stops with what is missing and the command that fixes it (`hw.sh profile sync`, `setup.linux.sh
--components repos`, `git submodule update --init`, `.devcontainer/prebuild.sh`, `BOAT_SIM_PLAYER=...`).
`test/test_workspace_derivation.py` checks the resolution against fixtures and the real workspace, and
`test_the_preview_and_sim_write_down_nothing_the_workspace_knows` (test/test_tool_help_hosts.py) fails on an image
name or hash, a ROS distro, a home path, a colcon call or a list of repos in these tools, their config or harness.
