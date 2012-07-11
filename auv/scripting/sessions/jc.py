import socket

from utils.watchfuncs import *

# processes running on both computers
processes = [
    Process('heimdall',   '{BDIR}',  zmq_daemon_pid(),       panic,      None,  ['{BDIR}/heimdall{D} -s']), 
    Process('daemon man', '{SDIR}',  node_pid('daemon_man'), restart(),  None,  ["sh -c 'PYTHONPATH=$PYTHONPATH:{SDIR} {SDIR}/run.sh {SDIR}/daemon_man.py --peer 10.0.0.2'"]),
    Process('scratch',    '{BDIR}',  node_pid('scratch'),    restart(),  None,  ['{BDIR}/scratch -n default/video']),
    Process('keepalive-wifi', '{BDIR}',  search_pid('telnet 10.0.0.4'),  restart(),  None,  ['telnet 10.0.0.4 7777']),
    Process('keepalive-eth',  '{BDIR}',  search_pid('telnet 10.0.0.2'),  restart(),  None,  ['telnet 10.0.0.2 7777']),
]

def get_processes(args):
    return processes
