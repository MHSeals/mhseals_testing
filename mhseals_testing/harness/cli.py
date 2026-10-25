"""boat_trial: the headless entry point (ROS-free until a command samples the boat).

`boat_trial check` evaluates the health checks and prints a table or JSON, so
an agent or CI can ask "is the boat ready" without the TUI. Exit codes: 0 ok,
1 warn (only with --fail-on warn), 2 fail, a blocked --gate, or bad input,
3 every check unknown (nothing was sampled), or a --require group was not evaluated.

``analyze NAME`` re-runs an offline analysis on a finished session and ``index`` rebuilds
INDEX.json, both ROS-free.
"""

import argparse
import json
import socket
import sys
import time
from fnmatch import fnmatchcase
from pathlib import Path

from mhseals_testing.health import config as health_config
from mhseals_testing.health.engine import Engine
from mhseals_testing.profiles import load_profile

TICK = 0.5          # seconds between evaluations while sampling
# boat_trial analyze NAME: the module whose main(argv) re-runs it on a finished session
ANALYSES = {'referee': 'mhseals_testing.harness.referee_analyze'}


def procedure_keys():
    """The real procedure keys, or None when the hardware package is not importable."""
    try:
        from mhseals_testing.procedures import PROCEDURES
    except ImportError:
        return None
    return {p.key for p in PROCEDURES}


def moves_boat(key):
    """Whether procedure ``key`` needs the boat armed; True when procedures cannot be imported."""
    try:
        from mhseals_testing.procedures import PROCEDURES
    except ImportError:
        return True
    return any(p.key == key and 'armed' in p.needs for p in PROCEDURES)


def select(specs, only):
    """Specs whose group (any case) or id (glob) matches one of ``only``; all when empty."""
    if not only:
        return list(specs)
    return [s for s in specs if any(
        s.group.lower() == w.lower() or fnmatchcase(s.id, w) for w in only)]


def no_samples():
    """Default collector: no probes exist yet, so every check reads as unknown."""
    return {}, {}, {}


def host_collector(bag_directory=None, clock=time.monotonic):
    """Collector for the probes that need no ROS: host load, temperature and disk.

    The graph, network, FCU/battery and perception probes need a live node and
    are not sampled here; their checks stay unknown (see ``unchecked``).
    """
    from mhseals_testing import data
    from mhseals_testing.health import probes
    store = probes.Facts(clock)
    bag_directory = bag_directory or data.root()      # the bag disk, never the working directory

    def collect():
        try:
            store.publish('host', probes.host_facts(bag_directory))
        except Exception as error:          # an unreadable /proc reads as unknown, not a crash
            store.publish('host', {}, error=repr(error))
        return (*store.read(), {})
    return collect


def unchecked(report):
    """Banner text when checks went unevaluated, else ''."""
    unknown, total = report['summary']['unknown'], len(report['checks'])
    if not total or not unknown:
        return ''
    if unknown == total:
        return f'NOTHING CHECKED: all {total} checks are unknown (no samples)'
    return f'{unknown} of {total} checks unknown (no sample from their probes)'


def run_check(config, profile_name, only=(), gate=None, fail_on='fail', window=5.0,
              collect=no_samples, clock=time.monotonic, sleep=time.sleep, host=None):
    """Sample for ``window`` seconds and return (report dict, exit code).

    ``collect()`` returns (facts, probes, session) as Engine.tick takes them.
    """
    specs = select(config.specs, only)
    engine = Engine(specs, periods={n: s['period'] for n, s in config.probes.items()
                                    if isinstance(s, dict) and 'period' in s}, clock=clock,
                    unknown_after=0.0)       # a one-shot check has no time to wait for evidence
    start = clock()
    while True:
        report = engine.tick(*collect())
        if clock() - start >= window:
            break
        sleep(TICK)
    keys = [gate] if gate else sorted({g for s in specs for g in s.gates if g != 'motion'})
    gates = {k: dict(zip(('blocking', 'warnings'), engine.gate(k, motion=moves_boat(k)))) for k in keys}
    checks = [{'id': s.id, 'group': s.group, 'status': report.results[s.id].status,
               'value': report.results[s.id].value, 'detail': report.results[s.id].detail,
               'hint': s.hint, 'data': report.results[s.id].data, 'age': None, 'source': s.kind}
              for s in specs if s.id in report.results]
    summary = {st: sum(c['status'] == st for c in checks) for st in ('ok', 'info', 'unknown', 'warn', 'fail')}
    code = 2 if summary['fail'] or (gate and gates[gate]['blocking']) else \
        1 if summary['warn'] and fail_on == 'warn' else 0
    return {'time': time.time(), 'host': host or socket.gethostname(), 'profile': profile_name,
            'summary': summary, 'gates': gates, 'checks': checks}, code


def required(report, groups):
    """{group: {evaluated, total}} for each ``--require`` group (any case)."""
    out = {}
    for group in groups:
        rows = [c for c in report['checks'] if c['group'].lower() == group.lower()]
        out[group] = {'evaluated': sum(c['status'] != 'unknown' for c in rows), 'total': len(rows)}
    return out


def table(report):
    rows = [f"{c['status']:<8}{c['id']:<28}{c['value']:<10} {c['detail']}" for c in report['checks']]
    counts = ' '.join(f'{n} {k}' for k, n in report['summary'].items() if n)
    return '\n'.join(rows + [counts or 'no checks'])


def build_parser():
    parser = argparse.ArgumentParser(prog='boat_trial', description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest='command', required=True)
    check = sub.add_parser('check', help='evaluate the health checks')
    check.add_argument('--profile', default='boat',
                       help='profile name or YAML path; see boat_trial profiles for descriptions')
    check.add_argument('--only', nargs='+', default=[], metavar='GROUP|ID',
                       help='group names or id globs to evaluate')
    check.add_argument('--window', type=float, default=5.0, help='seconds to sample')
    check.add_argument('--gate', metavar='PROC', help='fail when PROC cannot start')
    check.add_argument('--fail-on', choices=('warn', 'fail'), default='fail')
    check.add_argument('--json', metavar='PATH', help='write JSON to PATH, or - for stdout')
    check.add_argument('--lint', action='store_true', help='check the YAML only; needs no ROS')
    check.add_argument('--require', action='append', default=[], metavar='GROUP',
                       help='exit 3 unless GROUP has checks and at least one was evaluated')
    sub.add_parser('profiles', help='explain shipped profile command/sensor/output paths')
    course = sub.add_parser('course', help='compile course YAML to cached Unity JSON')
    course.add_argument('source', metavar='NAME|YAML')
    config = sub.add_parser(
        'config', help='parameter inventory; apply nav2 sets the running Nav2')
    verbs = config.add_subparsers(dest='config_command', required=True)
    catalog = verbs.add_parser('catalog')
    catalog.add_argument('--json', metavar='PATH|-', help='write JSON to PATH, or - for stdout')
    apply = verbs.add_parser(
        'apply', help='apply the operator nav2.yaml to the running Nav2 '
                      '(live, or with a Navigation reset)')
    apply.add_argument('target', choices=('nav2',))
    apply.add_argument('--yes', action='store_true',
                       help='apply; without it only the preview is printed')
    apply.add_argument('--now', action='store_true',
                       help='allow a live-only change while a goal runs '
                            '(the keys in config/nav2_classes.yaml)')
    apply.add_argument('--json', action='store_true', help='print one JSON document')
    analyze = sub.add_parser('analyze', help='re-run an offline analysis on a finished session')
    analyze.add_argument('analysis', choices=sorted(ANALYSES))
    analyze.add_argument('rest', nargs=argparse.REMAINDER, metavar='SESSION ...',
                         help='the analysis arguments (boat_trial analyze NAME -h)')
    index = sub.add_parser('index', help='rebuild INDEX.json, or summarize one session')
    index.add_argument('directory', nargs='?', metavar='DIR', help='a session or bag directory')
    index.add_argument('--json', action='store_true', help='print one JSON document')
    return parser


def session_muxed(registry):
    """Return whether the owner's session profile is muxed; None when unknown."""
    import yaml
    try:
        session = (registry.state() or {}).get('session')
        header = json.loads(Path(session, 'session.json').read_text())
        return load_profile(header['profile']).muxed
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError):
        return None


def cli_mux_expected(registry, record):
    """Say whether the mux must be up, from the fresh authority record and the profile.

    A guest cannot see the arm state: a mux-backed input, an rc or a hold in
    the record means the mux is in use. Without a record, assume it is.
    """
    if not isinstance(record, dict):
        return True
    if record.get('rc') or record.get('holds'):
        return True
    return record.get('input') is not None and session_muxed(registry) is not False


def apply_nav2(args, ports=None, **timing):
    """Preview ``config apply nav2``; with --yes set the running Nav2 from nav2.yaml."""
    from mhseals_testing import nav2_limits
    from mhseals_testing.harness import nav2_edit, nav2_live
    from mhseals_testing.harness.registry import Registry
    from mhseals_testing.history import History, adapters
    path = nav2_limits.path()
    try:
        raw = path.read_text() if path.exists() else ''
        content = nav2_limits.load(path, raw)
        requested = nav2_edit.flatten(content)
    except (OSError, ValueError) as error:
        print(f'boat_trial config apply: {path}: {error}', file=sys.stderr)
        return nav2_live.REFUSED
    shutdown = lambda: None  # noqa: E731
    if ports is None:
        from mhseals_testing.ros.nav2_live import RosPorts, cli_node, session_journal
        registry = Registry()
        node, shutdown = cli_node()
        box = []
        ports = RosPorts(node, registry, session_journal(registry),
                         lambda: cli_mux_expected(registry, box[0].authority()))
        box.append(ports)
    live = nav2_live.Nav2Live(ports, record=lambda: adapters.record_value(
        History(), 'nav2', 'cli:apply-nav2', content, raw, applied='live-session'), **timing)
    try:
        return nav2_live.cli_apply(requested, live, yes=args.yes, now=args.now,
                                   as_json=args.json)
    finally:
        shutdown()


def main(argv=None, collect=None):
    args = build_parser().parse_args(argv)
    if args.command == 'analyze':
        import importlib
        return importlib.import_module(ANALYSES[args.analysis]).main(args.rest)
    if args.command == 'index':
        from mhseals_testing.harness import index
        return index.main([*filter(None, [args.directory]), *(['--json'] if args.json else [])])
    if args.command == 'profiles':
        from mhseals_testing.profiles import profile_descriptions
        for name, description in profile_descriptions():
            print(f'{name}: {description}')
        return 0
    if args.command == 'course':
        import yaml
        from mhseals_testing.course import compile_course
        try:
            path = compile_course(args.source)
        except (OSError, ValueError, KeyError, ImportError, yaml.YAMLError) as error:
            print(f'boat_trial course: {error}', file=sys.stderr)
            return 2
        print(path)
        return 0
    if args.command == 'config' and args.config_command == 'apply':
        return apply_nav2(args)
    if args.command == 'config':
        from mhseals_testing import schema
        found, missing = schema.rows()
        keys = [dict(package=p, key=k, **e) for p, k, e in found]
        encoded = json.dumps(dict(missing=missing, keys=keys), indent=2)
        if args.json == '-':
            print(encoded)
        else:
            if args.json:
                with open(args.json, 'w') as output:
                    output.write(encoded + '\n')
            print(schema.text(found, missing))
        return 2 if missing else 0
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
    known = procedure_keys()
    if args.gate and known is not None and args.gate not in known:
        print(f'boat_trial: --gate {args.gate!r} is not a procedure key', file=sys.stderr)
        return 2
    report, code = run_check(config, profile.name, args.only, args.gate, args.fail_on,
                             args.window, collect or host_collector())
    banner = unchecked(report)
    if banner:
        print(f'boat_trial: {banner}', file=sys.stderr)
    if code == 0 and report['checks'] and report['summary']['unknown'] == len(report['checks']):
        code = 3                        # a pass that checked nothing must not look like a pass
    if args.require:
        report['required'] = required(report, args.require)
        unmet = [g for g, n in report['required'].items() if not n['evaluated']]
        if unmet:
            print(f'boat_trial: required group(s) not evaluated: {", ".join(unmet)}', file=sys.stderr)
            code = 2 if code == 2 else 3
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
