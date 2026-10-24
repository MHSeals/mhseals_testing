"""No offline file mutation may masquerade as acknowledged runtime correction."""
import pytest
from mhseals_hardware.menu import Cancelled
from mhseals_testing.harness.apply import CORRECTION_ACK_REQUIRED, RuntimeStaging, Staging
from mhseals_testing.procedures import PROCEDURES
from mhseals_testing.procedures.characterize import run
from test_app import make_app


def forbidden(*args, **kwargs):
    pytest.fail('unsupported correction reached motion/settings/prompt/file transition')


def test_shared_public_characterization_refuses_without_touching_context():
    records = []
    class RefusalContext:
        record = records.append
        def __getattr__(self, name):
            forbidden(name)
    result = run(RefusalContext())
    assert records == [result]
    assert result['verdict'] == 'REFUSED' and result['reason'] == CORRECTION_ACK_REQUIRED
    assert next(procedure for procedure in PROCEDURES if procedure.key == 'f').run is run


def test_runtime_adapter_refuses_raw_saved_restore_and_keeps_backup(tmp_path):
    from mhseals_hardware.configuration import update_config
    path = tmp_path / 'hardware.yaml'
    update_config({}, path)
    state = tmp_path / 'state.json'
    runtime = RuntimeStaging(path, state)
    with pytest.raises(Cancelled):
        runtime.raw_correction()
    assert not state.exists()
    offline = Staging(path, state)
    offline.raw_correction()
    before = path.read_bytes(), state.read_bytes()
    for transition in (runtime.restore, runtime.raw_correction,
                       lambda: runtime.commit([1., 0., 0., 0., 1., 0., 0., 0., 1.])):
        with pytest.raises(Cancelled):
            transition()
        assert (path.read_bytes(), state.read_bytes()) == before
    assert RuntimeStaging(path, state).blockers()  # durable across construction/restart


def test_tui_pending_restore_never_clears_any_motion_guard(tmp_path):
    app, _ = make_app(tmp_path)
    assert app.staging().blockers() == []
    offline = Staging(app.args.config)
    offline.raw_correction()
    before = offline.state.read_bytes()
    app.choose = forbidden
    app.offer_restore()
    app.apply_pending()
    assert app.notice == CORRECTION_ACK_REQUIRED
    assert offline.state.read_bytes() == before
    assert any('runtime C is unverified' in reason for reason in app.blockers())
    app.running = 'an unrelated procedure'
    assert any('runtime C is unverified' in reason for reason in app.blockers())
    app.arming.armed = True
    assert any('runtime C is unverified' in reason for reason in app.manual_blockers())
    restarted = RuntimeStaging(app.args.config)
    assert restarted.pending() and restarted.blockers()


def test_tui_bench_override_and_correction_shortcuts_cannot_bypass_refusal(tmp_path):
    app, _ = make_app(tmp_path)
    app.args.bench_override = True
    app.arm_panel = forbidden
    app.procedure_context = forbidden
    before = vars(app.settings).copy()
    for direction in ('', 'surge+'):
        app.characterize(direction)
        assert CORRECTION_ACK_REQUIRED in app.notice
        assert vars(app.settings) == before
    # Plain Drive has no new blocker when no backup exists.
    assert app.staging().blockers() == []
