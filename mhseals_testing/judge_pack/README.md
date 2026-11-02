# judge_pack: RoboBoat on the crane judge (Tier X1)

The crane judge core (crane_ml `judge/`) is robot- and tier-agnostic. This directory is the RoboBoat
side: Sources for every tier, the packs, a fast-tier replayer and a CLI. It is dev tooling, a
namespace directory (no `__init__.py`), so the boat install and the import-all smoke test skip it;
it needs a crane checkout with `judge/` (`CRANE_ROOT`, default `~/crane_ml`) and, for the replayer and
bags, a Python with `mcap`, `mcap_ros2` and crane `fastsim` (`~/.cache/fastsim-venv`).

| module | role |
|---|---|
| `sources.py` | kinematic and Unity harness sessions to judge records through the referee's own adapter (`analysis.run_record.load`); goals mapped to the course frame; Unity validity flags (`invalidWaterSearches`) make a run invalid and excluded from G3/G4, counted |
| `packs.py` | `roboboat`: `score.judge_record` unchanged (the body is byte-identical, tested), rows as metrics, `reliability.run_flags` as the outcome; `marine`: keep-off Property, speed profile, search share, time to the first execute decision, goals, interventions; the dependency graph and hints; the task with screening G4 bands |
| `replay.py` | reset-closed-loop replay of a higher tier's goal segments through the fast session's own calibrated plant (`fastsim.json`) |
| `pairs.py` | the fast and Unity pair archive (`~/.cache/fastsim-pairs`, read only) as pairs; a Unity twin shared by several pairs is one unit |
| `real.py` | the real boat from MCAP without ROS: odometry, GNSS (ENU, sigma, RTK status), `/cmd_vel`, journal and RTK events; odometry-vs-GNSS consistency, a first-order-plus-dead-time plant fit per axis against the fast tier's calibration |
| `nlc.py` | the NLC pilot outputs (`episodes.npz`, `summary.json`) as records, using the NLC evaluator's numbers as they are; arms paired by seed per family, D1 vs D2 block pairs per arm |

```sh
export CRANE_ROOT=~/crane_ml PYTHONPATH=$PWD:$CRANE_ROOT
PY=~/.cache/fastsim-venv/bin/python
$PY -m mhseals_testing.judge_pack pairs --out OUT            # fast vs Unity: G4, origin, decisions
$PY -m mhseals_testing.judge_pack session SESSION... --out OUT
$PY -m mhseals_testing.judge_pack real BAG_OR_SESSION --out OUT
$PY -m mhseals_testing.judge_pack nlc ~/nl-constraints-bench/out --out OUT
$PY -m pytest -q mhseals_testing/judge_pack/tests
```

Each command prints the one-line verdict and writes `report.json` (`crane.judge.report/1`),
`report.md`, `feed.json` (viewer anchors) and `verdict.txt`.
