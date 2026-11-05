"""The agent desk's constants: op classes, timings and the hint for each cancel or refusal cause."""

import re

MOTION = ('run', 'goal')
# a refusal of these is a footer notice too, not only an agent-log row
NOTICED = ('claim', 'run', 'goal', 'start', 'stop_part', 'set', 'fcu_set', 'fcu_restart', 'fcu_revert', 'arm',
           'reload', 'release_hold', 'fence')
URGENT = ('stop', 'release')   # the reqs a running act's blockers answer; the rest waits for the tick
LIFECYCLE_S = 2.0              # Nav2 lifecycle states are asked at most this often
LIFECYCLE_TIMEOUT_S = 0.1      # per server and step: a down Nav2 costs at most 1.2 s
DRY_TIMEOUT_S = 0.5            # costmap and plan waits of a goal dry run (on its own thread, never the tick)
FENCE_WAIT_S = 1.0             # off the tick only: the first wait for fence_watchdog's latched fence
DRY_CLOSE_S = 0.2              # closing the desk waits this long for a dry run before answering it cancelled
NOTES = 200
PART_OPS = {'start': 'start', 'stop_part': 'stop'}   # boat_agent start / stop COMPONENT
# boat_agent fcu ACTION: reads need no claim; writes and the restart need the agent's claim and a disarmed boat
FCU_OPS = ('fcu_get', 'fcu_rates', 'fcu_set', 'fcu_revert', 'fcu_restart')
FCU_CLAIMED = ('fcu_set', 'fcu_revert', 'fcu_restart')
# refused while the operator has declared manual, battery swap or power cycle (``paused``, exit 7); the reads,
# ask, note, mark, release and stop keep working
CHANGING = ('claim', 'run', 'goal', 'start', 'stop_part', 'set', 'arm', 'reload', 'release_hold', *FCU_CLAIMED)
# Never an agent's to start or stop, whatever agent.yaml lists: the thruster driver, the mux and the velocity
# controller (they arm), the operator's heartbeat and the safety watchdogs; nor a commander part (Nav2, the
# mission runner), checked by kind.
NEVER_PARTS = re.compile(r'thruster|command_mux|controller|operator_heartbeat|watchdog')
PROGRESS_S = 5.0
STATUS_S = 1.0
NAV2_PREFIX = 'nav2.'          # boat_agent set nav2.NODE.PARAM=V: a live Nav2 key (nav2/live.py)
ATTENTION_S = 30.0            # a refusal or a hold release keeps the header's "agent wants attention" this long
HINTS = {'keys': 'operator must re-arm and release the hold (U)', 'hold': 'operator must U',
         # Space's hold puts the mux on manual too, so 'keyboard' is Space or manual driving
         'keyboard': 'operator took over (Space or manual): wait until they re-arm and release the hold (U)',
         'rc': 'RC must return to auto', 'fault': 'operator must clear the fault and re-arm',
         'unknown': 'mux state is unknown',
         'geofence': 'fence hold: outside the fence or in its margin facing out; the operator drives inward, '
                     'then releases (U)',
         'costmap_stale': 'costmap hold: the costmap went stale; operator must U once it is fresh',
         'operator_link': 'operator link hold: operator must U once the link is back',
         'fcu_link': 'FCU link hold: operator must U once MAVROS is back',
         'ekf_inputs': 'EKF input hold: operator must U once the sensors are back',
         'approach': 'approach guard hold: operator must U',
         'stop file': 'the sim-arm stop file exists; only the operator removes it',
         'sim-arm timeout': 'the run passed agent.yaml sim_arm.max_s; it was disarmed',
         'motion observe': 'operator set Agent motion to observe: stopped and held; operator must U',
         'agents block': 'operator set Agents to block: stopped and held; operator must U'}
