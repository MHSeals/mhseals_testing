"""What one command's argv runs (wrappers, shells, ssh, docker, tmux: ``runs``, ``core``), the files it writes
(``written``) and what Python code runs from its literals (``python_runs``). Stdlib only."""

import os
import re

FIND_RUNS = ('-exec', '-execdir', '-ok', '-okdir', '-delete')

# --- what a call runs ----------------------------------------------------------------------------------------
# Shell keywords and wrappers run the words after them (their own options first: the letters in the value take the
# next word; POSITIONAL words more, such as timeout's duration). Shells (-c), su -c, script -c, watch, eval and tmux
# new/run take a command string; ssh and kin take a remote one; docker/podman run|exec and pixi/uv/conda run argv.
KEYWORDS = {'!', '{', '}', 'if', 'then', 'else', 'elif', 'do', 'while', 'until', 'coproc'}
WRAP = {'nohup': '', 'setsid': '', 'exec': 'a', 'command': '', 'builtin': '', 'time': 'fo', 'nice': 'n', 'env': 'uCS',
        'sudo': 'ugUhpCDrtTR', 'doas': 'uC', 'stdbuf': 'ioe', 'ionice': 'cnp', 'timeout': 'sk', 'chrt': '',
        'taskset': '', 'flock': 'wEc', 'xargs': 'aEdIiLlnPs', 'strace': 'eoOpPsuS', 'unbuffer': '', 'chroot': '',
        'systemd-run': 'pupE', 'firejail': '', 'nsenter': 'tSG', 'unshare': '', 'catchsegv': '', 'chronic': ''}
POSITIONAL = {'timeout': 1, 'chrt': 1, 'taskset': 1, 'flock': 1, 'chroot': 1}
SHELLS = {'bash', 'sh', 'dash', 'zsh', 'ksh', 'fish'}
SSH = {'ssh': 'bcDEeFIiJLlmOoPpQRSWw', 'autossh': 'MbcDEeFIiJLlmOoPpQRSWw', 'mosh': '', 'et': 'cpt'}
DOCKER = 'eEvlpuwhmc'     # docker/podman run|exec: short options that take a value (long ones: '--x=v' or the next word)
DOCKER_LONG = {'--env', '--volume', '--name', '--workdir', '--user', '--net', '--network', '--publish', '--entrypoint',
               '--env-file', '--mount', '--label', '--device', '--ipc', '--pid', '--gpus', '--runtime', '--add-host',
               '--hostname', '--cap-add', '--cap-drop', '--security-opt', '--shm-size', '--ulimit', '--memory',
               '--cpus', '--platform', '--restart', '--log-driver', '--detach-keys', '--group-add', '--tmpfs'}
TMUX_RUNS = {'new', 'new-session', 'new-window', 'neww', 'split-window', 'splitw', 'respawn-pane', 'respawnp',
             'respawn-window', 'respawnw', 'run-shell', 'run', 'display-popup', 'popup'}
ASSIGN = re.compile(r'[A-Za-z_]\w*=.*', re.S)


def options(words, takes, positional=0, assignments=False):
    """``words`` after a wrapper's options (``-x``; a letter in ``takes`` also takes the next word; ``--`` ends them),
    its VAR=x words (``assignments``: env, sudo) and ``positional`` words."""
    i = 0
    while i < len(words):
        word = words[i]
        if word == '--':
            i += 1
            break
        if word.startswith('-') and len(word) > 1:
            i += 1 + (len(word) == 2 and word[1] in takes)
        elif assignments and ASSIGN.fullmatch(word):
            i += 1
        else:
            break
    return words[i + positional:]


def runs(argv):
    """``(argv, text, remote)``: what ``argv`` runs as argv, as a command string, or on the ssh ``remote``."""
    name, rest = os.path.basename(argv[0]), argv[1:]
    if name in KEYWORDS:
        return rest, None, None
    if name in WRAP:
        return options(rest, WRAP[name], POSITIONAL.get(name, 0), name in ('env', 'sudo', 'doas')), None, None
    if name in SHELLS or name in ('su', 'script'):
        flag = next((i for i, w in enumerate(rest) if re.fullmatch(r'-[A-Za-z]*c|--command', w)), None)
        return None, rest[flag + 1] if flag is not None and flag + 1 < len(rest) else None, None
    if name in ('eval', 'watch'):
        return None, ' '.join(options(rest, 'nd') if name == 'watch' else rest), None
    if name == 'tmux':
        at = next((i for i, w in enumerate(rest) if w in TMUX_RUNS), None)
        if at is not None:
            words = options(rest[at + 1:], 'censtxyFlLSTb')
            return None, ' '.join(words) or None, None
        return None, None, None
    if name == 'sshpass':
        return options(rest, 'pfde'), None, None
    if name in SSH:
        words = options(rest, SSH[name])
        if name == 'et' and words:
            flag = next((i for i, w in enumerate(rest) if w == '-c'), None)
            return None, rest[flag + 1] if flag is not None and flag + 1 < len(rest) else None, words[0]
        if words[1:2] == ['--']:
            words = words[:1] + words[2:]
        return None, ' '.join(words[1:]) or None, words[0] if words else None
    if name in ('docker', 'podman') and rest[:1] in (['run'], ['exec'], ['container']):
        words, i = rest[1 + (rest[:1] == ['container']):], 0
        while i < len(words) and words[i].startswith('-'):
            i += 1 + ((len(words[i]) == 2 and words[i][1] in DOCKER) or words[i] in DOCKER_LONG)
        return words[i + 1:], None, None
    if name in ('pixi', 'uv', 'poetry', 'conda', 'mamba', 'pipenv', 'rye') and rest[:1] == ['run']:
        return options(rest[1:], 'nep'), None, None
    return None, None, None


def written(argv):
    """``[(op, path)]`` the files ``argv`` writes (op ``write`` or ``delete``): removers, movers, copies (the
    destination, and the file a copy into a directory makes), tee, truncate, chmod, dd of=, sed/perl -i and
    find -delete/-exec roots."""
    name, rest = os.path.basename(argv[0]), argv[1:]
    words = options(rest, {'cp': 'tS', 'mv': 'tS', 'ln': 'tS', 'install': 'tmogS', 'rsync': 'eTfB', 'truncate': 'sr',
                           'tee': '', 'sed': 'ef', 'perl': 'eEIM'}.get(name, ''))
    words = [w for w in words if not w.startswith('-')]
    if name in ('rm', 'unlink', 'shred', 'rmdir'):
        return [('delete', w) for w in words]
    if name in ('tee', 'truncate'):
        return [('write', w) for w in words]
    if name in ('chmod', 'chown', 'chgrp', 'chattr'):
        return [('write', w) for w in words[1:]]
    if name in ('cp', 'mv', 'ln', 'install', 'rsync'):
        into = next((w.split('=', 1)[1] if '=' in w else rest[i + 1] for i, w in enumerate(rest[:-1])
                     if w in ('-t', '--target-directory') or w.startswith('--target-directory=')), None)
        sources, dest = (words, into) if into else (words[:-1], words[-1] if words else None)
        if dest is None:
            return []
        out = [('delete', s) for s in sources] if name == 'mv' else []
        return out + [('write', dest)] + [('write', os.path.join(dest, os.path.basename(s.rstrip('/'))))
                                          for s in sources]
    if name == 'dd':
        return [('write', w[3:]) for w in rest if w.startswith('of=')]
    if name in ('sed', 'perl') and any(re.fullmatch(r'-[A-Za-z]*i\S*|--in-place\S*', w) for w in rest):
        script = any(w in ('-e', '-f', '-E') or re.fullmatch(r'-[A-Za-z]*e', w) for w in rest)
        return [('write', w) for w in (words if script else words[1:])]
    if name == 'find' and any(w in FIND_RUNS for w in rest):
        return [('delete', w) for w in (rest[:next((i for i, w in enumerate(rest) if w[:1] in '-(!'), len(rest))]
                                        or ['.'])]
    return []


PYTHON = re.compile(r'python[\d.]*$')
# Python calls that run a command (a literal string or list) and that write or remove a file (a literal path).
PY_RUNS = {'system', 'popen', 'run', 'call', 'check_call', 'check_output', 'Popen', 'getoutput', 'getstatusoutput',
           'create_subprocess_exec', 'create_subprocess_shell'}
PY_DELETES = {'remove', 'unlink', 'rmdir', 'rmtree', 'removedirs'}
PY_WRITES = {'write_text', 'write_bytes', 'truncate', 'touch', 'chmod'}
PY_MOVES = {'rename', 'replace', 'move', 'copy', 'copyfile', 'copy2', 'symlink', 'link'}


def python_runs(code):
    """``(argvs, texts, files)``: what Python ``code`` (``-c`` or on stdin) runs and writes, from its obvious
    literals only: ``subprocess.run([...])`` an argv, ``os.system('...')`` a command string, ``os.remove('...')``,
    ``Path('...').write_text`` or ``open('...', 'w')`` a ``(write|delete, path)``. A variable names nothing."""
    import ast
    try:
        tree = ast.parse(code)
    except (SyntaxError, ValueError):
        return [], [], []

    def text(node):
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            return node.value
        if isinstance(node, ast.Call) and name(node.func) in ('Path', 'PosixPath', 'PurePath') and node.args:
            return text(node.args[0])
        return None

    def name(func):
        return func.attr if isinstance(func, ast.Attribute) else func.id if isinstance(func, ast.Name) else ''

    argvs, texts, files = [], [], []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        called, args = name(node.func), node.args
        own = text(node.func.value) if isinstance(node.func, ast.Attribute) else None   # Path('x').unlink()
        paths = [p for p in [own] + [text(a) for a in args] if p is not None]
        if called in PY_RUNS and args:
            if isinstance(args[0], (ast.List, ast.Tuple)) and all(text(a) is not None for a in args[0].elts):
                argvs.append([text(a) for a in args[0].elts])
            elif called == 'create_subprocess_exec' and all(text(a) is not None for a in args):
                argvs.append([text(a) for a in args])
            elif text(args[0]) is not None:
                texts.append(text(args[0]))
        elif re.fullmatch(r'exec[lv]p?e?|spawn[lv]p?e?', called):
            lists = [[text(e) for e in a.elts] for a in args
                     if isinstance(a, (ast.List, ast.Tuple)) and all(text(e) is not None for e in a.elts)]
            words = [text(a) for a in args if text(a) is not None]       # execlp('ros2', 'ros2', 'topic', ...)
            argvs += lists[:1] or ([words[1:]] if len(words) > 1 else [])
        elif called in PY_DELETES:
            files += [('delete', p) for p in paths[:1]]
        elif called in PY_WRITES:
            files += [('write', p) for p in paths[:1]]
        elif called in PY_MOVES:
            files += [('delete', p) for p in paths[:1] if called in ('rename', 'replace', 'move')]
            files += [('write', p) for p in paths[1:2]]
        elif called == 'open' and paths:
            mode = next((text(k.value) for k in node.keywords if k.arg == 'mode'), None)
            mode = mode if mode is not None else (text(args[1]) if len(args) > 1 else 'r')
            files += [('write', paths[0])] if mode is None or set(mode) & set('wax+') else []
    return argvs, texts, files


def reads_stdin(argv):
    """``argv`` (a shell or python) takes its program from stdin: no ``-c``, no script, no ``-m``."""
    for word in argv[1:]:
        if word in ('-', '--', '-s') or not word.startswith('-'):
            return word in ('-', '-s')
        if re.fullmatch(r'-[A-Za-z]*[cm]', word):
            return False
    return True


def core(argv):
    """The program ``argv`` runs once its wrappers (sudo, env, timeout, ...) and VAR=x words are taken off."""
    words = list(argv)
    for _ in range(8):
        while words and ASSIGN.fullmatch(words[0]):
            words = words[1:]
        inner = runs(words)[0] if words else None
        if not inner:
            return words
        words = inner
    return words
