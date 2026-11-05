"""What App builds for itself (its composition root): the live feeds, the map views and the config watch.

Each function makes the same objects in the same order App's __init__ made them inline, so core.py coordinates
without importing every feed and view.
"""

from mhseals_hardware.configuration import config_path
from mhseals_testing.annotate.live import AnnotateFeed
from mhseals_testing.harness.watch import Watched, Watcher
from mhseals_testing.history import History, backends
from mhseals_testing.ui import keymap as keymap_module
from mhseals_testing.ui.app.help import map_key_maps
from mhseals_testing.ui.feeds import health as health_feed
from mhseals_testing.ui.feeds.conditions import ConditionsFeed
from mhseals_testing.ui.feeds.issues import IssuesFeed
from mhseals_testing.ui.feeds.rtk import RtkFeed
from mhseals_testing.ui.feeds.toasts import ToastFeed
from mhseals_testing.ui.feeds.tracking import TrackingFeed
from mhseals_testing.ui.minimap import panel as minimap_panel
from mhseals_testing.ui.minimap.layerbox import LayerBox
from mhseals_testing.ui.toasts import Toasts


def issues(given):
    """The device/part/transport problems feed: the one given, else a new one."""
    return given or IssuesFeed()


def live_feeds(app, profile):
    """Set ``tracking``, ``rtk``, ``annotations`` and ``feeds`` (ui.feeds objects ticked live, their view()
    merged into view()) on ``app``."""
    app.tracking = TrackingFeed()  # model ghost and live scores; session scores recorded at close
    app.rtk = RtkFeed(profile)
    app.annotations = AnnotateFeed()   # journals the automatic segment edges (annotate/live.py)
    app.feeds = [health_feed.HealthFeed(), app.tracking, ConditionsFeed(), app.rtk, app.issues, app.annotations]


def map_views(app):
    """Set the minimap, the layer box and the map and Fusion key maps on ``app``."""
    app.minimap = minimap_panel.MiniMap(app.map_state)
    app.layerbox = LayerBox(getattr(app.ros, 'map', None), app.map_state, live=True)
    files = app.state.files
    files.map_keys, files.map_tab_keys = map_key_maps(keymap_module.load()[0].doc)
    files.fusion_keys = keymap_module.key_map(keymap_module.load()[0].doc.get('tab.fusion'))


def toasts():
    return Toasts()          # health transitions feed it; never takes keys


def toast_feed(health):
    return ToastFeed(health)


def config_watch(app, args):
    """Set the hardware config file, its history backends and the watcher of the files App edits on ``app``."""
    app.config_file = config_path(args.config)
    app.history = History()
    app.config_backends = backends.file_backends(
        app.config_file, {'hardware': lambda: app.announce(app.watcher.reload('hardware'), observe=False)})
    app.watcher = Watcher()
    app.watcher.add(Watched('hardware', app.config_file, app.load_hardware, app.apply_hardware,
                            subsystem='hardware'))
