from types import SimpleNamespace as NS

from mhseals_testing.perception import transports, zed_diag

HEADER = ('  sl  local_address rem_address   st tx_queue rx_queue tr tm->when retrnsmt   uid  timeout inode\n')


def row(local, remote, state):
    return f'   0: {local} {remote} {state} 00000000:00000000 00:00000000 00000000  1000 0 1 1 0\n'


# 192.168.0.147 = 9300A8C0, 192.168.0.153 = 9900A8C0, 192.168.0.20 = 1400A8C0; 9090 = 2382, 7447 = 1D17
TABLE = (HEADER
         + row('9300A8C0:C350', '9900A8C0:2382', '01')      # blastoise -> squirtle rosbridge, ESTAB
         + row('9300A8C0:1D17', '1400A8C0:9C40', '01')      # zenoh peer .20 -> our :7447
         + row('9300A8C0:1D17', '9900A8C0:9C41', '01')      # zenoh peer .153
         + row('9300A8C0:1D17', '1700A8C0:9C42', '06')      # TIME_WAIT: not a peer
         + row('0100007F:1F90', '00000000:0000', '0A')      # listener
         + 'garbage line\n')


def test_parse_decodes_addresses_and_skips_noise():
    socks = transports.parse_proc_net_tcp(TABLE)
    assert len(socks) == 5
    assert socks[0] == ('192.168.0.147', 0xC350, '192.168.0.153', 9090, '01')
    assert socks[4].local_ip == '127.0.0.1' and socks[4].local_port == 8080


def test_rosbridge_needs_established_to_remote_port():
    socks = transports.parse_proc_net_tcp(TABLE)
    assert transports.rosbridge_connected(socks, '192.168.0.153')
    assert not transports.rosbridge_connected(socks, '192.168.0.20')
    closed = transports.parse_proc_net_tcp(HEADER + row('9300A8C0:C350', '9900A8C0:2382', '06'))
    assert not transports.rosbridge_connected(closed, '192.168.0.153')


def test_zenoh_peers_are_established_sorted_and_unique():
    socks = transports.parse_proc_net_tcp(TABLE)
    assert transports.zenoh_peers(socks) == ['192.168.0.153', '192.168.0.20']
    assert transports.zenoh_peers([]) == []


def test_read_proc_net_tcp_missing_file(tmp_path):
    assert transports.read_proc_net_tcp(tmp_path / 'absent') == []
    path = tmp_path / 'tcp'
    path.write_text(TABLE)
    assert len(transports.read_proc_net_tcp(path)) == 5


def status(level, message, pairs, hw='Stereolabs ZED 2i'):
    return NS(level=level, message=message, hardware_id=hw,
              values=[NS(key=k, value=v) for k, v in pairs])


def test_zed_diag_jp5_fixture():
    parsed = zed_diag.parse(status(0, 'Camera streaming', [
        ('Capture Mean Frequency', '14.9 Hz'), ('Capture Tot. Processing Time', '0.0123 sec'),
        ('Object detection Mean Frequency', '4.2 Hz'), ('Object detection Processing Time', '0.21 sec'),
        ('Serial Number', '12345')]))
    assert parsed['status'] == {'level': 'ok', 'message': 'Camera streaming'}
    assert parsed['stages']['camera'] == {'hz': 14.9, 'proc_ms': 12.3}
    assert parsed['stages']['sdk']['proc_ms'] == 210.0


def test_zed_diag_jp6_duplicates_last_wins_and_bad_values_skipped():
    parsed = zed_diag.parse(status(1, 'Low rate', [
        ('Capture Mean Frequency', '0 Hz'), ('Capture Mean Frequency', '14.7 Hz'),
        ('Object detection Processing Time', '18 msec'), ('Object detection Mean Frequency', 'N/A')]))
    assert parsed['status']['level'] == 'warn'
    assert parsed['stages'] == {'camera': {'hz': 14.7}, 'sdk': {'proc_ms': 18.0}}


def test_zed_diag_empty_and_hardware_id():
    assert zed_diag.parse(status(9, '', []))['status']['level'] == 'stale'
    assert zed_diag.parse(status(2, 'x', []))['stages'] == {}
    assert zed_diag.is_zed(status(0, '', [])) and not zed_diag.is_zed(status(0, '', [], hw='mavros'))
