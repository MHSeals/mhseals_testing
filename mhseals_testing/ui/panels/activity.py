"""Activity panel: the event log or the procedure results."""

from math import isfinite

from rich.console import Group
from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES, window
from mhseals_testing.ui.panels.scrubber import Loupe, label


def result_label(result):
    label = result.get('procedure', result.get('axis', '?'))
    if 'check' in result:
        label += f" {result['check']}"
    if 'direction' in result:
        label += ' +' if result['direction'] > 0 else ' -'
    return label


def result_detail(result):
    """The one number that matters for each kind of result."""
    for key, unit in (('response', ''), ('points', ' pts'), ('overshoot', ''),
                      ('cross_track_rms', ' m xtrack'), ('rms_radius', ' m radius')):
        value = result.get(key)
        if isinstance(value, (int, float)):
            if key == 'overshoot':
                return f'{value:.0%} overshoot'
            if key == 'points':
                return f"{value}/{result.get('possible', '?')}{unit}"
            return f'{value:+.3f}{unit}' if key == 'response' else f'{value:.2f}{unit}'
    return str(result.get('detail', result.get('status', '')))


def _subtitle(text, review):
    if not review:
        return text
    zoom = f'loupe {label(review.loupe)}{theme.SEPARATOR}' if _show_loupe(review) else ''
    return f'review{theme.SEPARATOR}{zoom}{text}'


LOUPE_ROWS = 5   # strip.rows form 'panel'


def _show_loupe(review):
    """Only spend Activity rows on a window narrower than the session overview."""
    if not review:
        return False
    duration = review.t1 - review.t0
    return (isfinite(duration) and isfinite(review.loupe)
            and 0 < review.loupe < duration)


def _with_loupe(body, review):
    return Group(Loupe(review), body) if _show_loupe(review) else body


def activity_panel(snapshot, view, height):
    focused = view['focus'] == 'activity'
    cursor = view['cursors'].get('activity', 0)
    budget = max(1, height - 3)
    review = view.get('review')
    review = review if review and not isinstance(review, bool) else None
    if _show_loupe(review):      # only a distinct local window takes rows from the list
        budget = max(1, budget - LOUPE_ROWS)
    if view['activity'] == 'log':
        table = theme.grid({'header': '  Event', 'ratio': 1}, {'header': 'Age', 'justify': 'right'},
                           padding=(0, 1))
        events = list(reversed(snapshot['events']))
        start, visible = window(events, cursor, budget)
        for index, (stamp, message) in enumerate(visible, start):
            table.add_row(theme.row(None, message, focused and index == cursor),
                          Text(f'{view["now"] - stamp:.0f}s', style=theme.MUTED))
        body = table if events else Text('No events yet', style=theme.MUTED)
        return theme.block(_with_loupe(body, review), TITLES['activity'], focused,
                           _subtitle('log', review))
    table = theme.grid({'header': '    Procedure', 'ratio': 3},
                       {'header': 'Result', 'width': 11},
                       {'header': 'Detail', 'ratio': 3}, padding=(0, 1))
    results = list(reversed(snapshot['results']))
    start, visible = window(results, cursor, budget)
    for index, result in enumerate(visible, start):
        verdict = str(result.get('verdict', theme.DASH))
        table.add_row(theme.row(verdict, result_label(result), focused and index == cursor),
                      Text(verdict.lower()), Text(result_detail(result), style=theme.MUTED))
    body = table if results else Text(f'No results yet{theme.SEPARATOR}Enter on a procedure runs it',
                                      style=theme.MUTED)
    return theme.block(_with_loupe(body, review), TITLES['activity'], focused,
                       _subtitle(f'results{theme.SEPARATOR}{len(results)}', review))
