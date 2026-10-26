"""The Pose popup (`o`, Enter on the pose row): estimates (the EKFs) and, below them, the sources feeding the boat.

``rows`` is computed only while the popup is open (App.odometry), from parameters fetched at
open, so a closed popup costs nothing.
"""
from mhseals_testing.analysis import fusion

ESTIMATES = ('ekf_local', 'ekf_global')


def rows(params_by_node, sources, sigmas=None):
    """(label, value) rows for the generic popup renderer.

    ``params_by_node`` maps an estimate to its parameter dict (None when unreadable);
    ``sources`` is [(name, state, value)]; ``sigmas`` maps estimate -> state -> {input: sigma}
    and adds a trust line only for states fused by two or more inputs.
    """
    out = [('Estimates', '')]
    for node in ESTIMATES:
        params = params_by_node.get(node)
        if params is None:
            out.append((node, 'parameters unavailable'))
            continue
        mode = '2D' if params.get('two_d_mode') else '3D'
        out.append((node, f"world {params.get('world_frame', '?')} {mode} {params.get('frequency', '?')} Hz"))
        inputs = fusion.inputs_from_params(params)
        for item in inputs:
            out.append((f'  {item.name}', f"{item.topic}  {' '.join(item.fused) or 'nothing fused'}"))
        for state, by_input in ((sigmas or {}).get(node) or {}).items():
            shares = fusion.trust(by_input)
            if shares:
                out.append((f'  {state}', ' '.join(f'{name} {share:.0%}' for name, share in shares.items())))
    out.append(('Sources', ''))
    out += [(name, f'{state} {value}'.strip()) for name, state, value in sources]
    return out
