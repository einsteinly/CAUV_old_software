from utils.watchfuncs import *

processes = [
    Process('img-pipeline',   '{BDIR}', node_pid('img-pipe'), panic,     None,
        ['{BDIR}/img-pipeline']
    ),
    Process('playlog', '{SDIR}', node_pid('py-play'),         panic,     None,
        ["sh -c 'sleep 6 && {SDIR}/run.sh ./playLog.py {SDIR}/camnav.chil -r 2.0'"]
    ),
    Process('slam-realdata', '{SDIR}', node_pid('py-slamr'), panic,     None,
        ['{SDIR}/tests/slam_realdata.py']
    ),
]

def get_processes(args):
    return processes

