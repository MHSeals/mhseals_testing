"""The README screenshots in docs/screens and the README's table of them match what the UI draws now
(`make -C docs screens` redraws them)."""
import base64
import hashlib
import os
import re
import subprocess
import sys
import zlib
from pathlib import Path

import pytest
import yaml

SCREENS = Path(__file__).resolve().parent.parent / 'docs' / 'screens'
PNG = re.compile(r'data:image/png;base64,([A-Za-z0-9+/=]+)')


def pixels_only(svg):
    """The SVG with each embedded PNG as its raw pixel rows: another zlib may compress them otherwise."""
    def raw(match):
        data = base64.b64decode(match.group(1))
        idat, i = b'', 8
        while i < len(data):
            size = int.from_bytes(data[i:i + 4], 'big')
            if data[i + 4:i + 8] in (b'IHDR', b'IDAT'):
                idat += data[i + 4:i + 8] + (zlib.decompress(data[i + 8:i + 8 + size]) if data[i + 4:i + 8] == b'IDAT'
                                             else data[i + 8:i + 8 + size])
            i += 12 + size
        return 'pixels:' + hashlib.sha256(idat).hexdigest()
    return PNG.sub(raw, svg)


def test_committed_screens_are_current(tmp_path):
    from importlib.metadata import version
    drawn = (SCREENS / 'rich-version.txt').read_text().strip()
    if version('rich') != drawn:
        pytest.skip(f'screens were drawn with rich {drawn}, this is rich {version("rich")}')
    env = ({k: v for k, v in os.environ.items() if k not in ('NO_COLOR', 'MHS_ASCII', 'TMUX')}
           | {'TERM': 'xterm-256color'})
    subprocess.run([sys.executable, str(SCREENS / 'render.py'), str(tmp_path)], check=True, env=env)
    stale = [e['name'] for e in yaml.safe_load((SCREENS / 'screens.yaml').read_text())
             if pixels_only((tmp_path / f"{e['name']}.svg").read_text())
             != pixels_only((SCREENS / f"{e['name']}.svg").read_text())]
    assert not stale, f'stale screens {stale}: run make -C docs screens and commit them'
    assert (tmp_path / 'README.md').read_text() == (SCREENS.parent.parent / 'README.md').read_text(), \
        'the README screens table differs from screens.yaml: run make -C docs screens'


def test_image_scenes_carry_their_tiles():
    """An image scene embeds the tiles; a text scene has none (its braille is the text)."""
    for e in yaml.safe_load((SCREENS / 'screens.yaml').read_text()):
        svg = (SCREENS / f"{e['name']}.svg").read_text()
        assert (PNG.search(svg) is not None) == (e.get('graphics') == 'image'), e['name']
