# Data layout

Every record the harness writes is JSON or JSONL under one root, owned by
`mhseals_testing/data.py`. No other module builds these paths.

## Root

1. `$MHSEALS_DATA`
2. `$ROS_WS/bags`
3. `~/astro_dock/bags`

The working directory is never used. `data.path(*parts)` joins under the root
and rejects absolute parts and `..`.

## Writing

| Call | Use |
|---|---|
| `write_json(path, record, schema)` | whole-file records (`session.json`, `runs/<id>.json`); atomic temp-file rename |
| `atomic_write(path, text)` | other text files (YAML, param snapshots) |
| `append_jsonl(path, record, schema=None)` | streams (`events.jsonl`, `health.jsonl`); one O_APPEND write plus fsync, lines under 4 KB |
| `read_json`, `read_jsonl` | readers; `read_jsonl` skips a torn last line |

Every JSON file carries a first `schema` field such as `mhseals.run/1`.
## Sessions today

`boat_test` still writes the flat layout, all under the root:

| Path | Content |
|---|---|
| `<profile>_<YYYYmmdd_HHMMSS>/` | the session bag (rosbag2, MCAP) |
| `<session>.report.json` | procedure results |
| `<session>.hardware.yaml` | the hardware config the driver ran with |
| `<session>.nav2.yaml` | Nav2 limits from a characterization |
| `<session>.session.yaml` | profile, clock domain (`wall` or `sim`) |
| `<session>.course_truth.json` | simulator course truth, copied at start |
| `logs/<session>/` | one log per child process |

`harness/session.py` creates `sessions/<YYYYMMDD_HHMMSS>_<host>_<profile>/` with
`session.json`, `logs/` and `snapshots/`, and lists the flat sessions above
read-only. `boat_test` is not switched to it yet.

## Index

`harness/index.py` rebuilds `sessions/<id>/summary.json`, the root `INDEX.json` and a generated
root `README.md` (layout and schemas, for agents) from `session.json`, the bag metadata,
`events.jsonl` and `runs/*.json`. There is no per-session `index.md`. Run
`python -m mhseals_testing.harness.index [DIR]`; `ensure()` rebuilds only when the index is
missing or older than a session file. Flags: `open`, `unfinalized`, `empty`, `bare`.


## Open a recorded session

In the Bags tab, select a closed session and press Enter. Review uses a separate
monitor fed by the segmented bag reader; arrows seek, `p` plays/pauses, and Esc
restores the previous source, tab and selected Bags row. The recorded host and
profile identify the review. Odometry details are read-only, and panels without
a replay feed remain empty rather than showing live observations.

Entry requires disarmed outputs, no running procedure/manual motion and no test
pulses. The selected recording or finalizing session is protected. Live callbacks,
recorder and process owners remain in place; their background work continues
against the live source. Review cannot arm, run procedures, release authority or
edit live controls. Space still requests the sticky hold and neutral/disarm while
pausing playback. Opening an invalid session leaves the live source intact and
shows a notice. Review does not initialize ROS or restart recording.
