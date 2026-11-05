"""The run popup: its rows for a run and the exclusion record (critique K4).

Pure helpers; ``App`` supplies the modal loop.
"""


def excluded(result):
    return 'excluded' in result


def exclusion(reason, by='tui'):
    """The ``excluded`` record for a run; a reason is required."""
    reason = reason.strip()
    if not reason:
        raise ValueError('an exclusion needs a reason')
    return dict(reason=reason, by=by)


def run_rows(result):
    """Label/value rows for the run popup: scalar fields, then the exclusion if any."""
    rows = [(name, value) for name, value in result.items()
            if not isinstance(value, dict)]
    if excluded(result):
        rows.append(('excluded', f"{result['excluded']['reason']} ({result['excluded']['by']})"))
    return rows
