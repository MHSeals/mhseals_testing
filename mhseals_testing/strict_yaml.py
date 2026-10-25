"""
The one strict reader for operator-edited YAML.

It refuses aliases, merge keys, duplicate or non-string keys, and non-finite
numbers; errors carry the file name (when given) and line number.
"""
import math

import yaml


class _Loader(yaml.SafeLoader):
    where = ''

    def _at(self, node):
        return f'{self.where}line {node.start_mark.line + 1}'

    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise ValueError(f'aliases are unsupported ({self._at(self.peek_event())})')
        return super().compose_node(parent, index)

    def construct_object(self, node, deep=False):
        value = super().construct_object(node, deep=deep)
        if isinstance(value, float) and not math.isfinite(value):
            raise ValueError(f'non-finite number ({self._at(node)})')
        return value

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key, _ in node.value:
            if key.tag != 'tag:yaml.org,2002:str':
                raise ValueError(f'keys must be strings, no merge keys ({self._at(key)})')
            if key.value in seen:
                raise ValueError(f'duplicate key {key.value!r} ({self._at(key)})')
            seen.add(key.value)
        return super().construct_mapping(node, deep=deep)


def load_strict(text, path=None):
    """
    Parse ``text``; ``path`` only labels errors.

    Raises ValueError on aliases, merge keys, duplicate or non-string keys and
    non-finite numbers; yaml.YAMLError (naming ``path``) on syntax errors.
    """
    loader = type('Loader', (_Loader,), {'where': f'{path}, ' if path else ''})(text)
    if path:
        loader.name = str(path)
    try:
        return loader.get_single_data()
    finally:
        loader.dispose()
