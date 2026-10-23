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
Layout and per-file schemas are summarised in `data_layout.md` of the plan and
in the generated `README.md` at the root.
