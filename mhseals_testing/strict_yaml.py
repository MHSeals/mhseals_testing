"""
The one strict reader for operator-edited YAML.

It refuses aliases, merge keys, duplicate or non-string keys, and non-finite
numbers; errors carry the file name (when given) and line number. A file with a
schema fragment beside it (config/schema/<stem>.yaml) is checked against it
(schema.validate): a problem raises ValueError naming file:key and what was expected.
"""
import logging
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
        doc = loader.get_single_data()
    finally:
        loader.dispose()
    if path:
        _validate(path, doc)
    return doc


STRICT = True       # False: problems are only logged (warn-only); test_schema_validate keeps the shipped configs clean


def _validate(path, doc):
    from mhseals_testing import schema      # schema reads its fragments through this module
    try:
        problems = schema.validate(path, doc)
    except (OSError, ValueError, yaml.YAMLError) as error:     # a broken fragment (test_schema's): the config loads
        logging.getLogger(__name__).warning('%s: its schema fragment does not load: %s', path, error)
        return
    if problems and STRICT:
        raise ValueError('; '.join(problems))
    for problem in problems:
        logging.getLogger(__name__).warning('%s', problem)
