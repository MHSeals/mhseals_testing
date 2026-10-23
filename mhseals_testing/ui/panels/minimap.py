"""The Drive minimap panel: frames the lines of ``view['minimap']`` (a ``ui.minimap.panel.MiniMap``)."""

from rich.text import Text

from mhseals_testing.ui import theme
from mhseals_testing.ui.panels import TITLES

WIDTH = 30     # cells, border and padding included (ui 4.5: minimap >= 30)
INSET = (4, 2)  # columns and rows the border and padding take


def minimap_panel(view, height):
    """The panel for ``height`` rows, or a no-feed note when the page has no minimap."""
    minimap = view.get('minimap')
    focused = view['focus'] == 'minimap'
    if minimap is None:
        return theme.block(Text('no map feed', style=theme.MUTED), TITLES['minimap'], focused)
    lines = minimap(WIDTH - INSET[0], max(1, height - INSET[1]))
    return theme.block(Text.from_ansi('\n'.join(lines)), TITLES['minimap'], focused, minimap.subtitle)
