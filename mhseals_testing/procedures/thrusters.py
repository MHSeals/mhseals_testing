"""Thruster mapping: mhseals_hardware's thruster_identify, run in the foreground.

Identification pulses each output alone, which the cmd_vel driver can't do,
so the outputs are disarmed first and the hardware tool owns them meanwhile.
"""


def command(profile, config_path, manual=False):
    argv = ['ros2', 'run', 'mhseals_hardware', 'thruster_identify',
            '--output-backend', profile.outputs]
    if config_path:
        argv += ['--config', str(config_path)]
    return argv + (['--manual'] if manual else [])


def run(ctx):
    mode = ctx.choose('Thruster mapping', ('pulse each output', 'enter the mapping by hand'),
                      {'p': 0, 'h': 1})
    services = ctx.services
    services['disarm']()
    returncode = services['foreground'](command(ctx.profile, services['user_config'],
                                                manual=mode != 'pulse each output'))
    services['reload_config']()
    result = dict(procedure='thruster mapping', mode=mode,
                  verdict='DONE' if returncode == 0 else f'EXITED {returncode}',
                  channel_map=list(services['hardware_config']['channel_map']))
    ctx.record(result)
    return result
