"""The agent hook (spec agent-control section 6): playbook, raw-motion denies, notes to the TUI.

astro_dock's ``.agents/hook.py`` loads this file (``src/<repo>/<package>/agent_hook.py``) and calls
``agent_event`` for Claude Code and Codex alike with one neutral event (``agent``, ``event``
start|pre|post, ``session_id``, ``tool_id``, ``command``, ``exit``, ``duration_ms``, ``failed``).
Imported under ``python3 -I`` on the dev machine: no ROS. PyYAML, when present, reads
config/agent.yaml (about 15 ms per call); without it the built-in defaults apply.

* ``start`` returns a brief (the agent's boat name, the one rule, where the playbook is); the
  whole playbook (the ``boat_agent`` verbs, exit codes) in a boat session or for ``--playbook``.
* ``pre`` matches what the call runs, never its text: each command of each pipeline (``$(...)``, ``bash -c``,
  wrappers such as sudo or timeout, and the remote command of ``ssh HOST ...`` included) becomes one line, its
  argv joined (``commands``), and each file it writes one ``EDIT op live|dev path`` line. ``hook.deny`` patterns
  match a line from its start; ``hook.deny_boat`` too when the line runs on a boat (this host is one, or ssh to
  one). Heredoc bodies, quoted text and the arguments of grep, echo or git commit are never commands, unless a
  shell, python or ssh reads them on stdin; ``python -c`` counts its obvious literals (``python_runs``).
  The reason names the command and the verb to use instead; the TUI gets it as ``phase: deny``. A
  boat-bound command (``hook.match``, not ``hook.skip``) becomes a channel ``note`` line
  ``{phase: pre, tool_id, cmd, agent}``.
* ``post`` sends the same tool_id with ``exit`` and ``dur_s`` (the CLI's duration, else since pre).

Lines go to this machine's channel (``agent/channel.jsonl`` under the data root, where a sim or
dev TUI reads) unless a boat session names its host: ``$BOAT_AGENT_TARGET`` (set for the agent
session that works on the boat), else ``hook.target`` (``local`` as shipped). Only then do they go
to ``hook.path`` on that host over a detached ``ssh ... cat >>`` (appended directly when this is
it), so sim and dev work never reaches the boat. At most ``hook.max_per_10s`` per session; the
excess folds into the next line as ``folded``. Delivery failures go to ``$XDG_STATE_HOME/boat_agent_hook.log`` and never reach
the agent. Advisory, not a boundary: the mux, the claim and the red external chip are the safety.
"""

import hashlib
import importlib
import importlib.util
from pathlib import Path
import sys
import types

PARTS = ('rules', 'shell', 'runs', 'hosts', 'notes', 'playbook', 'scan', 'event')   # hook/, re-exported here


def package():
    """hook/ beside this file: ``mhseals_testing.hook`` when imported with the package, else loaded by path under a
    private name (the dispatcher loads this file alone under ``python3 -I``; relative imports in hook/ need a parent)."""
    if __package__:
        return importlib.import_module(__package__ + '.hook')
    here = Path(__file__).resolve().parent / 'hook'
    name = '_agent_hook_' + hashlib.sha1(str(here).encode()).hexdigest()[:12]
    if name not in sys.modules:
        spec = importlib.util.spec_from_file_location(name, here / '__init__.py', submodule_search_locations=[str(here)])
        sys.modules[name] = module = importlib.util.module_from_spec(spec)
        try:
            spec.loader.exec_module(module)
        except BaseException:
            del sys.modules[name]
            raise
    return sys.modules[name]


_PKG = package()
_MODULES = [importlib.import_module(f'{_PKG.__name__}.{part}') for part in PARTS]
for _module in _MODULES:
    globals().update({k: v for k, v in vars(_module).items() if not k.startswith('__')})


class _Entry(types.ModuleType):
    """Setting a name on this module (a test's monkeypatch of ``hostname`` or ``network_file``) sets it in every hook/
    module that holds the same object, so the code that moved there sees it as it did here."""

    def __setattr__(self, name, value):
        old = self.__dict__.get(name, _Entry)
        for module in _MODULES:
            if old is not _Entry and module.__dict__.get(name, _Entry) is old:
                setattr(module, name, value)
        super().__setattr__(name, value)


if getattr(sys.modules.get(__name__), '__dict__', None) is globals():
    sys.modules[__name__].__class__ = _Entry
