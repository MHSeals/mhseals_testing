"""boat_trial: the headless entry point (ROS-free until a command samples the boat).

`boat_trial check` evaluates the health checks and prints a table or JSON, so
an agent or CI can ask "is the boat ready" without the TUI. Exit codes: 0 ok,
1 warn (only with --fail-on warn), 2 fail, a blocked --gate, or bad input.
"""

import argparse
from fnmatch import fnmatchcase
import json
import socket
import sys
import time

from mhseals_testing.health import config as health_config
from mhseals_testing.health.engine import Engine
from mhseals_testing.profiles import load_profile

TICK = 0.5          # seconds between evaluations while sampling


def procedure_keys():
    """The real procedure keys, or None when the hardware package is not importable."""
    try:
        from mhseals_testing.procedures import PROCEDURES
    except ImportError:
        return None
    return {p.key for p in PROCEDURES}


def select(specs, only):
    """Specs whose group (any case) or id (glob) matches one of ``only``; all when empty."""
    if not only:
        return list(specs)
    return [s for s in specs if any(
        s.group.lower() == w.lower() or fnmatchcase(s.id, w) for w in only)]


def no_samples():
    """Default collector: no probes exist yet, so every check reads as unknown."""
    return {}, {}, {}


def run_check(config, profile_name, only=(), gate=None, fail_on='fail', window=5.0,
              collect=no_samples, clock=time.monotonic, sleep=time.sleep, host=None):
    """Sample for ``window`` seconds and return (report dict, exit code).

    ``collect()`` returns (facts, probes, session) as Engine.tick takes them.
    """
    specs = select(config.specs, only)
    engine = Engine(specs, periods={n: s['period'] for n, s in config.probes.items()
                                    if isinstance(s, dict) and 'period' in s}, clock=clock)
    start = clock()
    while True:
        report = engine.tick(*collect())
        if clock() - start >= window:
            break
        sleep(TICK)
    keys = [gate] if gate else sorted({g for s in specs for g in s.gates if g != 'motion'})
    gates = {k: dict(zip(('blocking', 'warnings'), engine.gate(k))) for k in keys}
    checks = [{'id': s.id, 'group': s.group, 'status': report.results[s.id].status,
               'value': report.results[s.id].value, 'detail': report.results[s.id].detail,
               'hint': s.hint, 'data': report.results[s.id].data, 'age': None, 'source': s.kind}
              for s in specs if s.id in report.results]
    summary = {st: sum(c['status'] == st for c in checks) for st in ('ok', 'info', 'unknown', 'warn', 'fail')}
    code = 2 if summary['fail'] or (gate and gates[gate]['blocking']) else \
        1 if summary['warn'] and fail_on == 'warn' else 0
    return {'time': time.time(), 'host': host or socket.gethostname(), 'profile': profile_name,
            'summary': summary, 'gates': gates, 'checks': checks}, code


def table(report):
    rows = [f"{c['status']:<8}{c['id']:<28}{c['value']:<10} {c['detail']}" for c in report['checks']]
    counts = ' '.join(f'{n} {k}' for k, n in report['summary'].items() if n)
    return '\n'.join(rows + [counts or 'no checks'])


def build_parser():
    parser = argparse.ArgumentParser(prog='boat_trial', description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('check', help='evaluate the health checks')
    check.add_argument('--profile', default='boat')
    check.add_argument('--only', nargs='+', default=[], metavar='GROUP|ID',
                       help='group names or id globs to evaluate')
    check.add_argument('--window', type=float, default=5.0, help='seconds to sample')
    check.add_argument('--gate', metavar='PROC', help='fail when PROC cannot start')
    check.add_argument('--fail-on', choices=('warn', 'fail'), default='fail')
    check.add_argument('--json', metavar='PATH', help='write JSON to PATH, or - for stdout')
    check.add_argument('--lint', action='store_true', help='check the YAML only; needs no ROS')
    return parser


def main(argv=None, collect=no_samples):
    args = build_parser().parse_args(argv)
    try:
        profile = load_profile(args.profile)
        config = health_config.load(profile=profile)
    except (health_config.ConfigError, OSError, ValueError, KeyError) as error:
        print(f'boat_trial: {error}', file=sys.stderr)
        return 2
    if args.lint:
        problems = health_config.lint(config.specs, procedure_keys(), config.unmatched, config.probes)
        print('\n'.join(problems) or f'{len(config.specs)} checks, no problems')
        return 2 if problems else 0
    if args.window < 0:
        print('boat_trial: --window must not be negative', file=sys.stderr)
        return 2
    report, code = run_check(config, profile.name, args.only, args.gate, args.fail_on,
                             args.window, collect)
    text = json.dumps(report, indent=2, default=str)
    if args.json == '-':
        print(text)
    else:
        if args.json:
            with open(args.json, 'w') as out:
                out.write(text + '\n')
        print(table(report))
    return code


if __name__ == '__main__':
    sys.exit(main())
