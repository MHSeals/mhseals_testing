# Health

Health answers "is the boat fit to run?" from named facts and YAML checks. It is advisory plus start gates: a `fail` stops a procedure from starting, a `warn` shows its reason, and nothing blocks motion mid-run (the real interlocks stay in `readiness.motion_blockers`, `Plant.conflicts` and arming).

```
probes (I/O)  ->  facts {name: Fact}  ->  kinds (pure)  ->  Result  ->  Engine (hysteresis, cap)  ->  HealthReport
```

Code lives in `mhseals_testing/health/`: `model.py` (types), `registry.py` (kinds), `engine.py`, `config.py` (YAML loading and lint). The shipped kinds are `threshold` (a number against bands, or a value against `expect`), `state` (a text fact that must be one of the expected values), `topic_rate`, `endpoints`, `nodes`, `log_pattern`, `params`, `git_state`, `processes`, `pwm_owner` (who drives the thrusters) and `pwm_readback` (what the PWM outputs do); the shipped checks are the files in `config/health/`.

## Facts

A `Fact(value, label, data, time)` is one named measurement, for example `battery.cell_v`. A probe publishes facts; a kind reads them. Kinds never do I/O, so tests pass plain dicts. A signal that needs no new judgement is a new fact plus a `threshold` entry, not a new kind. Derived quantities (a speed disagreement, minutes of disk left) are computed in the probe, never in YAML.

## Adding a kind

Add a module to `mhseals_testing/health/kinds/`. It opens with a docstring saying what question it answers and which incident motivated it, defines a frozen `Params` dataclass (its fields are the YAML keys; unknown or missing keys fail when the config loads) and registers a pure function:

```python
@kind('threshold', Params, needs=(), fields=('value',))
def evaluate(params, facts):
    return Result('ok', value='3.9 V', detail='battery.cell_v 3.9 V')
```

`needs` names the probes the kind reads, `watch(params)` returns per-check probe targets, and `fields` lists the `result.data` keys message templates may use. The registry imports every module in `kinds/`, so there is nothing else to register.

## Adding a check

Checks live in `config/health/*.yaml`:

```yaml
group: Power                  # default popup group for the file
defaults: {severity: warn}    # any check key
checks:
  - id: power.cell_voltage    # unique, dotted lowercase
    kind: threshold
    fact: battery.cell_v      # kind parameters, inline
    warn: 3.5
    fail: 3.3
    unit: ' V'
    severity: fail
    title: Cell voltage
    message: 'Lowest cell {value}'
```

Check keys: `id`, `kind`, `group`, `title`, `severity` (the cap: `info`, `warn` or `fail`), `show` (inline anchors the check may colour), `gates` (procedure keys, or `motion`), `when` (equality over session facts, else the check is ok), `missing` (`unknown`, `skip` or `fail` when a fact is absent), `raise_after`, `clear_after`, `message`, `hint`, `profiles` (empty: all), `enabled`, `advisory` (the dashboard advisory row the check folds into; checks sharing a name make one row, the worst wins), and `each` (a list of rows, one check per row, merged over the entry). `{field}` templates fill from the row, the profile and, in `message` and `hint`, the result.

Anchors for `show`: `rose`, `thrusters`, `header.deploy`, `odometry` and `sensor:<name>`.

## Engine rules

- `Engine.tick(facts, probes, session)` evaluates every enabled check and returns a `HealthReport(results, transitions)`.
- A non-ok condition becomes active after `raise_after` (default 2 s for warn, immediate for fail) and clears after `clear_after` (default 3 s).
- The shown status is `min(result.status, severity)`. Statuses sort `ok < info < unknown < warn < fail`.
- A fact whose probe raised or went stale gives `unknown` (following `missing`), never a stale value.
- Transitions `(id, from, to, result, time)` feed toasts and the bag.

## User overrides

`~/.config/mhseals/health.yaml` may `disable` ids (globs allowed), `override` fields of one id and set `probes:` options. Precedence: shipped file, user file, session mute. An entry that matches no check is a lint problem.

## Lint

`config.lint(specs, procedures, unmatched)` returns the problems as strings: unique ids, known kinds and anchors, compiling regexes, templates that only name known fields, and gate keys that are real procedures. The shipped files must lint clean (`test/test_health_shipped.py`).

## Budget

Tick-thread probes must stay cheap (about 2 ms per tick on the ODROID); blocking work runs on one worker thread and the popup footer shows the measured cost.
