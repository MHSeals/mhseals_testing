"""DiffLive writes only the screen rows that changed, nothing for an unchanged frame, everything after a resize."""
import io

from rich.console import Console
from rich.text import Text

from mhseals_testing.ui.difflive import SYNC_BEGIN, DiffLive

KNOBS = {'full_every': 0.0, 'sync': True}


def screen(rows):
    return Text('\n'.join(rows))


def make(width=20, height=4):
    out = io.StringIO()
    console = Console(file=out, force_terminal=True, width=width, height=height, color_system='standard')
    clock = [0.0]
    live = DiffLive(screen(['a', 'b', 'c', 'd']), console=console, screen=True, auto_refresh=False,
                    clock=lambda: clock[0], settings=dict(KNOBS))
    return live, out, clock


def test_only_changed_rows_are_written_and_an_unchanged_frame_writes_nothing():
    live, out, _ = make()
    with live:
        first = out.getvalue()
        assert all(f'\x1b[{y};1H' in first for y in range(1, 5))     # the first frame is the whole screen
        out.truncate(0), out.seek(0)
        live.update(screen(['a', 'B', 'c', 'd']), refresh=True)
        sent = out.getvalue()
        assert '\x1b[2;1H' in sent and '\x1b[1;1H' not in sent and '\x1b[3;1H' not in sent
        assert sent.startswith(SYNC_BEGIN) and live.sent == len(sent)
        out.truncate(0), out.seek(0)
        live.update(screen(['a', 'B', 'c', 'd']), refresh=True)
        assert out.getvalue() == '' and live.sent == 0


def test_invalidate_the_full_period_and_a_resize_send_the_whole_screen():
    live, out, clock = make()
    same = screen(['a', 'b', 'c', 'd'])

    def sent_rows():
        out.truncate(0), out.seek(0)
        live.update(same, refresh=True)
        return sum(f'\x1b[{y};1H' in out.getvalue() for y in range(1, 5))
    with live:
        assert sent_rows() == 0
        live.invalidate()
        assert sent_rows() == 4
        live.settings['full_every'] = 1.0
        clock[0] = 5.0
        assert sent_rows() == 4 and sent_rows() == 0
        live.console.width = 30
        assert sent_rows() == 4


def test_outside_a_terminal_it_is_rich_live():
    out = io.StringIO()
    console = Console(file=out, force_terminal=False, width=20, height=4)
    with DiffLive(screen(['x']), console=console, screen=True, auto_refresh=False, settings=dict(KNOBS)) as live:
        live.update(screen(['y']), refresh=True)
    assert '\x1b[1;1H' not in out.getvalue()
