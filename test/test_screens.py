"""The README screenshots in docs/screens match what the UI draws now (`make -C docs screens` redraws them)."""
import os
import subprocess
import sys
from pathlib import Path

import yaml

SCREENS = Path(__file__).resolve().parent.parent / 'docs' / 'screens'


def test_committed_screens_are_current(tmp_path):
    env = {k: v for k, v in os.environ.items() if k not in ('NO_COLOR', 'MHS_ASCII', 'TMUX')} | {'TERM': 'xterm-256color'}
    subprocess.run([sys.executable, str(SCREENS / 'render.py'), str(tmp_path)], check=True, env=env)
    stale = [e['name'] for e in yaml.safe_load((SCREENS / 'screens.yaml').read_text())
             if (tmp_path / f"{e['name']}.svg").read_text() != (SCREENS / f"{e['name']}.svg").read_text()]
    assert not stale, f'stale screens {stale}: run make -C docs screens and commit them'
