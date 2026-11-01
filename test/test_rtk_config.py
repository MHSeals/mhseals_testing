"""Where rtk_base keeps its settings and surveys."""


def test_the_config_lives_in_the_mounted_operator_config(tmp_path, monkeypatch):
    from mhseals_testing import rtk
    monkeypatch.delenv('MHSEALS_RTK_CONFIG', raising=False)
    monkeypatch.setenv('XDG_CONFIG_HOME', str(tmp_path))
    assert rtk.config_path() == tmp_path / 'mhseals/rtk.yaml'
    (tmp_path / 'mhseals_testing').mkdir()
    (tmp_path / 'mhseals_testing/rtk.yaml').write_text('{}\n')
    assert rtk.config_path() == tmp_path / 'mhseals_testing/rtk.yaml'      # the old one, while alone
    (tmp_path / 'mhseals').mkdir()
    (tmp_path / 'mhseals/rtk.yaml').write_text('{}\n')
    assert rtk.config_path() == tmp_path / 'mhseals/rtk.yaml'
