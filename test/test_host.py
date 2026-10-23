from types import SimpleNamespace

from mhseals_testing.harness.host import context
from mhseals_testing.ui import popups


def test_context_binds_the_host_methods():
    host = SimpleNamespace(**{n: object() for n in ('motion', 'prompt', 'choose', 'record', 'supervise', 'observe')})
    ctx = context(host, 'p', 'm', 's', 3.0, {'x': 1})
    assert (ctx.prompt, ctx.motion, ctx.observe) == (host.prompt, host.motion, host.observe)
    assert ctx.stale_after == 3.0 and ctx.services == {'x': 1} and ctx.profile == 'p'


def test_choose_returns_the_shortcut_option():
    app = SimpleNamespace(popup=lambda modal, handle: handle('y'), scroll=popups.scroll)
    assert popups.choose(app, 'Arm', ['Arm', 'Cancel'], {'y': 0, 'n': 1}) == 'Arm'
