"""The agent desk, re-exported: it lives in ui/feeds/agent_desk/ and its contract ("Safety, in one place") is
agent_desk/desk.py's docstring. ``from mhseals_testing.ui.feeds import agent`` and every name below keep working."""

from mhseals_testing.ui.feeds.agent_desk.common import (  # noqa: F401 - re-exported
    MOTION, NOTICED, URGENT, LIFECYCLE_S, LIFECYCLE_TIMEOUT_S, DRY_TIMEOUT_S, FENCE_WAIT_S, DRY_CLOSE_S, NOTES,
    PART_OPS, FCU_OPS, FCU_CLAIMED, CHANGING, NEVER_PARTS, PROGRESS_S, STATUS_S, NAV2_PREFIX, ATTENTION_S, HINTS)
from mhseals_testing.ui.feeds.agent_desk.desk import (  # noqa: F401
    AgentDesk, blockers, ended, run_act, start, step_setting)
from mhseals_testing.ui.feeds.agent_desk.popups import operator_popup  # noqa: F401 - re-exported
