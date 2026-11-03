"""config/annotate.yaml: the digit tags, the quick phrases and the steady-window thresholds."""

from functools import lru_cache
from pathlib import Path

from mhseals_testing.strict_yaml import load_strict

EFFECTS = ('none', 'degraded', 'suspect', 'invalid')     # worst last


def path():
    source = Path(__file__).resolve().parents[2] / 'config' / 'annotate.yaml'
    if source.exists():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('mhseals_testing')) / 'config' / 'annotate.yaml'


@lru_cache(maxsize=1)
def load(file=None):
    """The validated settings: ``tags`` {digit str: {label, effect, span}}, ``phrases`` {digit str: text}."""
    file = Path(file or path())
    doc = load_strict(file.read_text(), file)
    tune = doc['tune']
    tags = {str(n): dict(v) for n, v in enumerate(tune['tags'][:9], 1)}
    for digit, tag in tags.items():
        if tag.get('effect') not in EFFECTS or len(tag.get('span', ())) != 2:
            raise ValueError(f'{file}: tag {digit} needs an effect in {EFFECTS} and a [before, after] span')
    return {'pick': float(tune['pick']), 'tags': tags, 'phrases': {str(n): str(v) for n, v in enumerate(tune['phrases'][:9], 1)},
            'steady': dict(tune['steady']), 'scales': tuple(doc['scales'])}


def tag(label):
    """The tag entry named ``label``; unknown labels are an operator's own words: no effect."""
    return next((t for t in load()['tags'].values() if t['label'] == label),
                {'label': label, 'effect': 'none', 'span': [0, 0]})


def worst(effects):
    return max(effects, key=EFFECTS.index, default='none')
