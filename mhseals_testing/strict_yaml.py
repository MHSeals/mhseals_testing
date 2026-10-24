"""The one strict reader for operator-edited YAML: no aliases, merge keys, duplicate or non-string keys."""
import yaml


class _Loader(yaml.SafeLoader):
    def compose_node(self, parent, index):
        if self.check_event(yaml.AliasEvent):
            raise ValueError('aliases are unsupported')
        return super().compose_node(parent, index)

    def construct_mapping(self, node, deep=False):
        seen = set()
        for key, _ in node.value:
            where = f'line {key.start_mark.line + 1}'
            if key.tag != 'tag:yaml.org,2002:str':
                raise ValueError(f'keys must be strings, no merge keys ({where})')
            if key.value in seen:
                raise ValueError(f'duplicate key {key.value!r} ({where})')
            seen.add(key.value)
        return super().construct_mapping(node, deep=deep)


def load_strict(text):
    """Parse ``text``; raise ValueError (yaml.YAMLError included) on anything ambiguous."""
    return yaml.load(text, Loader=_Loader)
