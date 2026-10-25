"""Health engine package: config, model and registry load without ROS or hardware.

The advisory power/FCU/clock checks live in ``health.legacy``.

Health answers "is the boat fit to run?" from named facts and YAML checks. It is advisory plus
start gates: a ``fail`` stops a procedure from starting, a ``warn`` shows its reason, and nothing
blocks motion mid-run (the interlocks stay in ``readiness.motion_blockers``, ``Plant.conflicts``
and arming).

    probes (I/O) -> facts {name: Fact} -> kinds (pure) -> Result -> Engine -> HealthReport

* Facts: a probe publishes ``Fact(value, label, data, time)``; kinds read them and never do I/O.
  A signal needing no new judgement is a new fact plus a ``threshold`` check, not a new kind.
  Derived quantities are computed in the probe, never in YAML.
* Kinds: a module in ``kinds/`` with a docstring (the question and the incident behind it), a
  frozen ``Params`` dataclass (its fields are the YAML keys) and a pure function registered with
  ``@kind(name, Params, needs=probes, fields=message-template keys)``. The registry imports
  every module in ``kinds/``.
* Checks: ``config/health/*.yaml`` (``group``, ``defaults``, ``checks``; keys are the fields of
  ``model.CheckSpec``). ``each`` expands one check per row. ``~/.config/mhseals/health.yaml`` may
  ``disable`` ids (globs allowed), ``override`` fields of one id and set ``probes:`` options;
  an entry matching no check is a lint problem. ``config.lint`` must pass on the shipped files.
* Engine: a non-ok result becomes active after ``raise_after`` and clears after ``clear_after``;
  the shown status is ``min(result.status, severity)`` (``unknown`` is not capped); a stale or
  failed probe gives ``unknown`` per ``missing``.
* Tick-thread probes must stay cheap (about 2 ms per tick on the ODROID); blocking work runs on
  one worker thread.
"""
