# Sourced by boat_tui and boat_agent: the vessel registry facts their help and targets read, never a host name
# written here. The registry is $ASTRO_REGISTRY, else the one astro_dock resolves (hw.sh whoami --registry);
# hosts and roles are astro_dock's devices.yaml, read with its own loader (devices/profile.py, stdlib only).
astro_ws=${ASTRO_WS:-$(cd "$(dirname "$0")/../../.." 2>/dev/null && pwd)}
if [ ! -x "$astro_ws/hw.sh" ]; then          # a worktree: the workspace of its main clone
    common=$(git -C "$(dirname "$0")" rev-parse --path-format=absolute --git-common-dir 2>/dev/null)
    [ -n "$common" ] && astro_ws=$(cd "$common/../../.." 2>/dev/null && pwd)
fi

registry() {
    if [ -n "${ASTRO_REGISTRY:-}" ]; then echo "$ASTRO_REGISTRY"
    elif [ -x "$astro_ws/hw.sh" ]; then "$astro_ws/hw.sh" whoami --registry 2>/dev/null
    fi
}

# role_host ROLE...: the first registry host that holds one of ROLES (tried in order); fails without one.
role_host() {
    reg=$(registry)
    [ -f "$reg" ] && [ -f "$astro_ws/devices/profile.py" ] || return 1
    python3 -I -c '
import importlib.util, sys
spec = importlib.util.spec_from_file_location("astro_profile", sys.argv[1])
profile = importlib.util.module_from_spec(spec)
spec.loader.exec_module(profile)
hosts = profile.registry_hosts(sys.argv[2])
for role in sys.argv[3:]:
    for name, host in hosts.items():
        if role in ((host or {}).get("roles") or ()):
            sys.exit(print(name))
sys.exit(1)' "$astro_ws/devices/profile.py" "$reg" "$@" 2>/dev/null
}

# The vessel the registry belongs to (its profile directory's name), or nothing.
vessel() {
    reg=$(registry)
    [ -n "$reg" ] && basename "$(dirname "$reg")"
}

# The host the boat's TUI runs on: the tui_backend role, else the boat computer.
tui_host() { role_host tui_backend boat_computer; }

# The dev container boat_test runs in: the registry's defaults.container (hook/registry.py dev_container), or nothing.
dev_container() {
    python3 -I -c '
import importlib.util, sys
spec = importlib.util.spec_from_file_location("testing_registry", sys.argv[1])
registry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(registry)
print(registry.dev_container())' "$(dirname "$0")/../mhseals_testing/hook/registry.py" 2>/dev/null
}
