"""Shell text to commands: heredocs out, words and operators, pipelines with their stdin, the files they write
and their ``$(...)``, backtick and ``<(...)`` substitutions (``parse``). Stdlib only."""

import re
import shlex

def plain(text):
    """``text`` with continuations joined and quotes and escapes dropped (the note filter, ``matches``)."""
    return text.replace('\\\n', '').replace("'", '').replace('"', '').replace('\\', '')


def matches(patterns, text):
    """Any pattern found in ``plain(text)``; ``.`` spans newlines (the TUI-note filter)."""
    text = plain(text)
    return any(re.search(p, text, re.S) for p in patterns)


SEPARATORS = {';', '&', '&&', '||', '|', '|&', '\n', ';;', '(', ')'}
REDIRECTS = {'<', '>', '>>', '>|', '&>', '&>>', '>&', '<&', '<>'}
PUNCT = re.compile(r'<<<|<<|>>|>&|<&|<>|&>>|&>|>\||\|\||&&|\|&|;;|[\s\S]')
HEREDOC_OP = re.compile(r'(?<!<)<<(?!<)(-?)[ \t]*')


def delimiter(line, start):
    """The heredoc word at ``line[start:]`` as bash reduces it: quotes and backslashes removed, and any of them
    makes the body literal. ``(word, quoted)``; ValueError for a word the hook will not guess at."""
    word, quoted, i = '', False, start
    while i < len(line) and line[i] not in ' \t;&|()<>':
        c = line[i]
        if c in '\'"':
            end = line.find(c, i + 1)
            if end < 0 or (c == '"' and '\\' in line[i + 1:end]):
                raise ValueError('heredoc delimiter')
            word, quoted, i = word + line[i + 1:end], True, end + 1
        elif c == '\\':
            if i + 1 >= len(line):
                raise ValueError('heredoc delimiter')
            word, quoted, i = word + line[i + 1], True, i + 2
        else:
            word, i = word + c, i + 1
    if not word or re.search(r'[$`]', word):
        raise ValueError('heredoc delimiter')
    return word, quoted


def heredocs(text):
    """``(lines, bodies)``: the command lines of ``text`` (backslash-newline joined) with each heredoc body taken
    out, and ``bodies`` ``[[dash, word, quoted, body, line index, terminator]]``. ValueError on any doubt."""
    raw, lines, found, i = text.split('\n'), [], [], 0
    while i < len(raw):
        line = raw[i]
        i += 1
        while line.endswith('\\') and i < len(raw):
            line, i = line[:-1] + raw[i], i + 1
        lines.append(line)
        for op in HEREDOC_OP.finditer(line):
            word, quoted = delimiter(line, op.end())
            dash, body = bool(op.group(1)), []
            while i < len(raw) and (raw[i].lstrip('\t') if dash else raw[i]) != word:
                body.append(raw[i])
                i += 1
            if i >= len(raw):
                raise ValueError('unterminated heredoc')
            found.append([dash, word, quoted, '\n'.join(body), len(lines) - 1, raw[i]])
            i += 1
    return lines, found


def tokens_of(text):
    """Shell words and operators; a quoted word stays one word and ``\\;`` a word, not a separator (find -exec,
    tmux). Raises ValueError on an open quote."""
    lexer = shlex.shlex(text.replace('\\;', '\x01'), posix=True, punctuation_chars=';&|()<>`\n')
    lexer.whitespace, lexer.commenters, lexer.whitespace_split = ' \t\r', '', True
    out = []
    for token in lexer:
        out += PUNCT.findall(token) if set(token) <= set(';&|()<>`\n') else [token.replace('\x01', '\\;')]
    return out


def parse(text, bodies=None):
    """``[pipeline [command {argv, stdin, writes, subs, reads}]]`` of ``text``. Heredoc bodies come out of the
    text first (``bodies`` shared with nested substitutions); ``subs`` are ``$(...)``, backticks and ``<(...)``."""
    if bodies is None:
        lines, found = heredocs(text)
        bodies, text = iter([(f[3], f[2]) for f in found]), '\n'.join(lines)
    return build(tokens_of(text), bodies)


def substitutions(text):
    """The commands of each ``$(...)`` (parentheses balanced) and backtick pair in a word."""
    out, i = [], 0
    while i < len(text):
        if text.startswith('$(', i) and not text.startswith('$((', i):
            depth, j = 1, i + 2
            while j < len(text) and depth:
                depth += (text[j] == '(') - (text[j] == ')')
                j += 1
            out.append(text[i + 2:j - (depth == 0)])
            i = j
        elif text[i] == '`' and '`' in text[i + 1:]:
            end = text.index('`', i + 1)
            out.append(text[i + 1:end])
            i = end + 1
        else:
            i += 1
    return out


def build(tokens, bodies):
    pipelines, pipe = [], []
    cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': [], 'reads': []}

    def end(piped):
        nonlocal cmd, pipe
        if any(cmd.values()):
            pipe.append(cmd)
        cmd = {'argv': [], 'stdin': [], 'writes': [], 'subs': [], 'reads': []}
        if not piped and pipe:
            pipelines.append(pipe)
            pipe = []

    def nested(text):
        for inner in substitutions(text):
            try:
                cmd['subs'].append(parse(inner, bodies))
            except ValueError:
                cmd['subs'].append([[{'argv': inner.split(), 'writes': [], 'subs': []}]])

    i = 0
    while i < len(tokens):
        token = tokens[i]
        i += 1
        process = token in ('<', '>') and tokens[i:i + 1] == ['(']           # <(...) and >(...) run their command
        if token == '`' or process or (token == '(' and cmd['argv'] and cmd['argv'][-1].endswith('$')):
            i += process
            depth, j = 1, i
            while j < len(tokens) and depth:
                depth += (-(tokens[j] == '`') if token == '`' else (tokens[j] == '(') - (tokens[j] == ')'))
                j += 1
            inner = tokens[i:j - (depth == 0)]
            cmd['subs'].append(build(inner, bodies))
            cmd['argv'] += [t for t in inner if t not in SEPARATORS]     # `kill $(pgrep x)` names x
            i = j
        elif token in ('|', '|&'):
            end(True)
        elif token in SEPARATORS:
            end(False)
        elif token == '<<':
            i += 1                                                       # the delimiter word
            body, quoted = next(bodies, ('', True))
            cmd['stdin'].append(body)
            if not quoted:                                               # $(...) in it runs
                nested(body)
        elif token == '<<<' or token in REDIRECTS:
            if cmd['argv'] and cmd['argv'][-1].isdigit():               # 2>/dev/null: the fd, not a word
                cmd['argv'].pop()
            target = tokens[i] if i < len(tokens) else ''
            if token == '<<<':
                cmd['stdin'].append(target)
                nested(target)                                           # <<< "$(...)" runs it
            elif token == '<':
                cmd['reads'].append(target)
            elif token not in ('<', '<&') and target != '/dev/null' and not (
                    token == '>&' and re.fullmatch(r'\d|-', target)):
                cmd['writes'].append(target)                             # a file written (not 2>&1)
            i += 1
        else:
            cmd['argv'].append(token)
            nested(token)                                                # "$(...)" inside quotes
    end(False)
    return pipelines
