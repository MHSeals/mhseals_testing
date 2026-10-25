"""The Health popup (``!``): every check that is not ok, worst first, refreshed while open."""

ORDER = {'fail': 0, 'warn': 1, 'unknown': 2}


def rows(engine):
    """(status, title · detail) per non-ok, unmuted check, worst first; one 'all checks ok' row otherwise."""
    results = getattr(engine, 'last', None) or {}
    muted = getattr(engine, 'muted', ())
    out = [(ORDER.get(result.status, 3), result.status, spec.title or spec.id, result.detail)
           for spec in getattr(engine, 'specs', ()) if spec.id not in muted
           for result in [results.get(spec.id)] if result is not None and result.status not in ('ok', 'info')]
    return ([(status.upper(), f'{title} · {detail}' if detail else title) for _, status, title, detail in sorted(out)]
            or [('OK', 'all checks ok')])


def open_health(app):
    """Show the popup; Esc or q closes."""
    m = dict(kind='message', title='Health', rows=rows(app.engine), width=96, hint='Esc close')
    app.popup(m, lambda key: True if key in ('escape', 'q') else None,
              lambda: m.update(rows=rows(app.engine)))
