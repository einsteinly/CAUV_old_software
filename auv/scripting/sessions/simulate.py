from utils.watchfuncs import *

processes = [
    Process('vehicle_daemon',  '/',       zmq_daemon_pid(),     'HIGH',    ['{BDIR}/vehicle_daemon{D}']),
    Process('control',         '/',       node_pid('Control'),  'HIGH',    ['{BDIR}/control{D} -N']),
    Process('sim.py',          '{SDIR}',  node_pid('py-sim'),   'MEDIUM',  ['sh -c \'PYTHONPATH={SDIR} python2.7 {SDIR}/simulator/sim.py\'']),
    Process('persist',         '{SDIR}',  node_pid('persist'),  'LOW',     ['{SDIR}/persist.py -rs']),
]

def get_processes(args):
    return processes
