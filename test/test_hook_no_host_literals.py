"""No machine facts in the agent hook's code (D-20261104-03): its boat hosts come from the device registry, so a renamed
or added boat can never slip past a name written here. Every string constant of agent_hook.py and hook/*.py (docstrings
and f-string parts too) is checked against the registry's host and vessel names and the ``NAME-odroid|jetson|orin``
shape. Comments are not code and stay free to give an example."""

import ast
import os
from pathlib import Path
import re

from mhseals_testing import agent_hook as hook

import hook_registry

PACKAGE = Path(hook.__file__).resolve().parent
SHAPE = re.compile(r'[a-z]+-(odroid|jetson|orin)')


def files():
    return [PACKAGE / 'agent_hook.py'] + sorted((PACKAGE / 'hook').glob('*.py'))


def strings(path):
    """``(line, text)`` of every string constant in ``path``."""
    return [(node.lineno, node.value) for node in ast.walk(ast.parse(path.read_text()))
            if isinstance(node, ast.Constant) and isinstance(node.value, str)]


def names():
    """Host and vessel names: the shipped profile set (hook_registry), the package devices.yaml and, when this machine
    has one, the workspace's own profile set; never the package's fallback host ``local``."""
    found = set()
    for profiles in (hook_registry.SHIPPED, hook_registry.THREE):
        for vessel, (profile, devices) in profiles.items():
            found |= {vessel} if 'kind: vessel' in profile else set()
            found |= {name for name, _ in hook.registry_hosts(devices or '')}
    found |= {name for name, _ in hook.registry_hosts((PACKAGE.parent / 'config' / 'devices.yaml').read_text())}
    root = Path(os.environ.get('ASTRO_PROFILES') or Path.home() / 'astro_dock' / 'profiles')
    for profile in root.glob('*/profile.yaml') if root.is_dir() else ():
        text = profile.read_text()
        found |= {profile.parent.name} if hook.top(text, 'kind') == 'vessel' else set()
        registry = hook.top(text, 'registry')
        if registry and (profile.parent / registry).is_file():
            found |= {name for name, _ in hook.registry_hosts((profile.parent / registry).read_text())}
    return found - {'local'}


def test_the_hook_names_no_host_or_vessel():
    known = names()
    assert {'blastoise', 'blastoise-odroid', 'squirtle-jetson', 'tranquility'} <= known
    word = re.compile(r'(?<![\w.-])(' + '|'.join(map(re.escape, sorted(known, key=len, reverse=True))) + r')(?![\w-])')
    found = [f'{path.name}:{line}: {text[:60]!r}' for path in files() for line, text in strings(path)
             if word.search(text) or SHAPE.search(text)]
    assert not found, 'host or vessel names in the hook (read them from the registry):\n' + '\n'.join(found)


def test_the_check_sees_a_literal_host():
    """The check itself: a literal like the 11-03 one is found, in a string and in a docstring."""
    tree = ast.parse('"""Reaches blastoise-odroid."""\nHOSTS = [f"{x}", "squirtle-jetson"]\n')
    texts = [n.value for n in ast.walk(tree) if isinstance(n, ast.Constant) and isinstance(n.value, str)]
    assert sum(bool(SHAPE.search(t)) for t in texts) == 2
