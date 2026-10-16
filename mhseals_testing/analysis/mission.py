"""Score a task run from its /task/events against a rubric (docs/math.tex, "Mission scoring").

A rubric names the events a good run must produce (with minimum counts), the
points each event is worth, and a time limit. The score is the points earned
for counted events, capped per event, and the run passes when it finished
successfully, within the limit, with every required event present.
"""

from collections import Counter


def score(events, rubric, outcome=None):
    counts = Counter(event['event'] for event in events)
    finished = next((e for e in reversed(events) if e['event'] == 'task_finished'), None)
    outcome = outcome or (finished or {}).get('outcome', 'INCOMPLETE')
    elapsed = (finished or (events[-1] if events else {})).get('elapsed', 0.0)
    required = rubric.get('required', {})
    missing = {name: count - counts[name] for name, count in required.items()
               if counts[name] < count}
    # An event counts once unless the rubric caps it higher (max_counts).
    caps = {name: rubric.get('max_counts', {}).get(name, 1) for name in rubric.get('points', {})}
    earned = {name: points * min(counts[name], caps[name])
              for name, points in rubric.get('points', {}).items() if counts[name]}
    limit = rubric.get('time_limit')
    late = limit is not None and elapsed > limit
    passed = outcome == 'SUCCESS' and not missing and not late
    return dict(outcome=outcome, elapsed=elapsed, events=dict(counts), missing=missing,
                points=sum(earned.values()), points_by_event=earned,
                possible=sum(points * caps[name] for name, points in rubric.get('points', {}).items()),
                late=late, verdict='PASS' if passed else 'FAIL')
