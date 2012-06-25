from utils.watchfuncs import *

processes = [
    Process('vehicle_daemon',  '/',       zmq_daemon_pid(),     panic,      None,    ['{BDIR}/vehicle_daemon{D} -s']),
    Process('control',         '/',       node_pid('Control'),  restart(),  'root',  ['{BDIR}/control{D} -N']),
    Process('sim.py',          '{SDIR}',  node_pid('py-sim'),   restart(),  None,    ['sh -c \'PYTHONPATH={SDIR} python2.7 {SDIR}/simulator/sim.py\'']),
    Process('persist',         '{SDIR}',  node_pid('persist'),  restart(),  None,    ['{SDIR}/persist.py -rs -f {SDIR}/persist/sim']),
    Process('imgpipeline',     '/',       node_pid('img-pipe'), restart(),  None,    ['{BDIR}/img-pipeline -n ai']),
]

def get_processes(args):
    return processes
