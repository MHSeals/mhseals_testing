# Quarantine opt-in (mhseals_agents README): the workspace agent tools' pytest plugin runs quarantined tests as
# xfail; a no-op when that package or its plugin is not importable (<workspace>/src not on PYTHONPATH, an old clone).
_spec = __import__('importlib.util').util.find_spec
pytest_plugins = ['mhseals_agents.pytest_quarantine'] if _spec('mhseals_agents') and _spec('mhseals_agents.pytest_quarantine') else []
