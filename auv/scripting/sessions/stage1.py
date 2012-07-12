import socket

from utils.watchfuncs import *

# processes running on both computers
common_processes = [
    Process('heimdall',       '/',       zmq_daemon_pid(),         panic,      None,  ['{BDIR}/heimdall{D} -s']),
]

if True or socket.gethostname() == 'barracuda-seco':
    # processes running on seco only
    processes = common_processes + [
        Process('bridge',     '/',       search_pid('mcb_bridge'), restart(),  None, ['{BDIR}/mcb_bridge -p /dev/ttyS0 -n 3 --prefix /tmp/ttyV']),
        Process('control',    '/',       node_pid('control'),      restart(),  None, ["sh -c 'sleep 1.0 && {BDIR}/control{D} --sbg /tmp/ttyV1 --imu sbg --mcb barracuda --port /tmp/ttyVCAN0'"]),
        Process('persist',    '{SDIR}',  node_pid('persist'),      restart(),  None, ['{SDIR}/persist.py -rs -f {SDIR}/persist/sim']),
        Process('daemon_man', '{SDIR}',  node_pid('daemon-man'),   restart(),  None, ['{SDIR}/daemon_man.py --peer 10.0.0.3']),
        Process('last resort','{SDIR}', search_pid('last-resort.py'),restart(),None, ['{SDIR}/last-resort.py 7777 300']),
    ]
else:
    # processes running on the panda board only
    processes = common_processes + [
        Process('daemon_man',  '{SDIR}',  node_pid('daemon-man'),   restart(), None,  ['{SDIR}/daemon_man.py -peer 10.0.0.2']),
    ]

def get_processes(args):
    return processes
