"""RoboBoat on the crane fast-sim core and the crane cluster.

  adapter   the domain adapter: mhseals_nav's MissionHost drives the core's navigator; runs are
            harness sessions the referee scores unchanged (crane fastsim/MIGRATION.md)
  config    config/fastsim_roboboat.yaml (tune on top) -> the core's config
  batch     ``python3 -m mhseals_testing.fastsim_adapter.batch``: seeds -> scored run folders
  cluster   crane-worker backends: ``mhseals_testing.fastsim_adapter.cluster:FastsimBackend`` and
            ``...cluster:ReplayBackend``
  replay    bag replay: a session's recorded inputs through tracker and mission logic offline,
            diffed against what the boat did
  bag       MCAP sessions without ROS: decoded messages and a TF buffer
"""
