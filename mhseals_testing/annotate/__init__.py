"""Field annotation: what the operator, agents and the TUI mark during a session, and the manifest built from it.

    config    config/annotate.yaml: digit tags, quick phrases, steady-window thresholds
    fold      the session journal folded into marks, segments, conditions and language commands (review edits applied)
    manifest  the versioned session manifest (``manifest.json`` beside the bag) and its loader for consumers
    live      the feed that journals automatic segments each tick (fix tier, holds, takeovers, steady commands)

Everything is append-only journal lines (``events.jsonl``); the manifest is a pure fold of them, so it can be
rebuilt at any time and review edits never rewrite what was recorded. ROS-free.
"""
