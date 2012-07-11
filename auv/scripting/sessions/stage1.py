import socket

from utils.watchfuncs import *

# processes running on both computers
common_processes = [
    Process('heimdall',       '/',       zmq_daemon_pid(),         panic,      None,  ['{BDIR}/heimdall{D} -s']),
]

if True or socket.gethostname() == 'barracuda-seco':
    # processes running on seco only
    processes = common_processes + [
        Process('bridge',     '/',       search_pid('mcb_bridge'), restart(), 'root', ['{BDIR}/mcb_bridge -p /dev/ttyS0 -n 3 --prefix /tmp/cauv/ports/ttyV']),
        Process('control',    '/',       node_pid('control'),      restart(), 'root', ["sh -c 'sleep 2.0 && {BDIR}/control{D} --sbg /tmp/cauv/ports/ttyV1 --imu sbg --mcb barracuda --port /tmp/cauv/ports/ttyVCAN'"]),
        Process('persist',    '{SDIR}',  node_pid('persist'),      restart(),  None,  ['{SDIR}/persist.py -rs -f {SDIR}/persist/sim']),
        Process('daemon man', '{SDIR}',  node_pid('daemon_man'),   restart(),  None,  ['{SDIR}/daemon_man.py --peer 10.0.0.3']),
        Process('last resort','{SDIR}', search_pid('last-resort.py'),restart(),None,  ['{SDIR}/last-resort.py 7777 300']),
    ]
else:
    # processes running on the panda board only
    processes = common_processes + [
        Process('daemon_man',  '{SDIR}',  node_pid('daemon_man'),   restart(), None,  ['{SDIR}/daemon_man.py -peer 10.0.0.2']),
    ]

def get_processes(args):
    return processes
