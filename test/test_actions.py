"""The action table: labels and footer hint priorities."""

from mhseals_testing.ui.actions import ACTIONS


def test_actions_are_well_formed():
    assert ACTIONS
    for action_id, action in ACTIONS.items():
        assert action_id and action_id == action_id.strip()
        assert action.label.strip(), action_id
        assert isinstance(action.hint, int) and action.hint >= 0, action_id
