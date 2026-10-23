"""The run popup and the Enter routing of timeline items (timeline.md TL5, critique K4).

Pure helpers: which owner an item opens, the popup rows for a run, and the
exclusion record. ``App`` supplies the modal loop and the destinations.
"""

# Item kind (the ts095 Item kinds, plus hold and cluster) -> owner opened by Enter.
ROUTES = {
    'run': 'run', 'abort': 'run', 'proc': 'run', 'manual': 'run',
    'config': 'history',   # H popup, preselected on that id
    'health': 'health',    # popup.health on its History tab
    'mark': 'marker',      # note text or a show popup
    'auth': 'show',        # who, pid, host, reason, duration
    'hold': 'show',
    'cluster': 'choose',   # members to pick from
}


def route(item):
    """The owner (``run``, ``history``, ``health``, ``marker``, ``show``, ``choose``) for an item."""
    return ROUTES.get(item.get('kind'), 'show')


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
