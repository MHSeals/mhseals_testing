"""The Status sidebar with many unreachable parts (11-01 boat): every Components name stays readable, no host
name or ``state on host`` text sits in the column, and short of lines those parts fold into one ``net`` line."""

import os
import subprocess
import sys
from pathlib import Path

import pytest

TESTS = Path(__file__).parent

PAGE = """
import sys
from types import SimpleNamespace
sys.path.insert(0, sys.argv[1])
from test_dashboard import monitor, text, view
from mhseals_testing.profiles import load_profile
from mhseals_testing.ui import parts
from mhseals_testing.ui.layout import render

items = {f'p{i}': SimpleNamespace(key=f'p{i}', title=f'Part {i}', part=f'p{i}', profile='boat',
                                  remote='blastoise-jetson' if i % 2 else 'blastoise-odroid') for i in range(12)}
items['odometry'] = SimpleNamespace(key='odometry', title='Odometry', part='odometry', profile='boat',
                                    remote='blastoise-jetson')
stack = SimpleNamespace(items=items, by={}, start_order=lambda mark: [],
                        state=lambda key, nodes: ('off', '') if key == 'odometry' else ('unknown', ''))
issues = SimpleNamespace(on_row=lambda item: [] if item.key == 'odometry' else [f'{item.remote} does not answer'])
profile = load_profile('boat')
width, height = int(sys.argv[2]), int(sys.argv[3])
snapshot = monitor().snapshot(profile.required_tf)
snapshot['components'] = parts.rows(stack, [], issues)
sys.stdout.write(text(render(snapshot, view(profile), width, height), width, height))
"""


def page(width, height, **env):
    base = {key: value for key, value in os.environ.items() if key not in ('MHS_ASCII', 'NO_COLOR', 'TERM')}
    result = subprocess.run([sys.executable, '-c', PAGE, str(TESTS), str(width), str(height)],
                            env={**base, 'TERM': 'xterm', **env}, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
    return result.stdout


def sidebar(out):
    """The Status column: the cells of each line up to the panel's right edge."""
    lines = out.splitlines()
    top = next(i for i, line in enumerate(lines) if 'Status' in line)
    edge = lines[top].index('╮' if '╮' in lines[top] else '+', lines[top].index('Status'))
    return [line[:edge + 1] for line in lines[top:]]


@pytest.mark.parametrize('size', [(80, 24), (120, 40), (200, 55)])
@pytest.mark.parametrize('env', [{}, {'NO_COLOR': '1'}, {'MHS_ASCII': '1'}], ids=['16', 'mono', 'ascii'])
def test_unreachable_parts_never_crowd_out_the_components(size, env):
    width, height = size
    out = page(width, height, **env)
    assert all(len(line) <= width for line in out.splitlines())
    column = '\n'.join(sidebar(out))
    assert 'blastoise' not in column and 'unknown' not in column and ' on ' not in column
    for name in ('Components', 'pose', 'FCU pose', 'Odometry', 'Localization'):
        assert name in column, (name, column)
    if env.get('MHS_ASCII'):
        assert out.isascii()
    if height < 40:              # short of lines: the twelve unreachable parts are one line
        assert 'net' in column and ('12 unreachable' in column or '12 down' in column)
        assert 'Part 3' not in column
    else:                        # room for them all: each part is its name and glyphs only
        assert all(f'Part {i}' in column for i in range(12)) and 'net' not in column
