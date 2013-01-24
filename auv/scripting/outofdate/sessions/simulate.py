#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from utils.watchfuncs import *

processes = [
    Process('heimdall',        '/',       zmq_daemon_pid(),     restart(),  None,    ['heimdall -s']),
    Process('control',         '/',       node_pid('Control'),  restart(),  None,  ['control -N --mcb barracuda']),
    Process('sim.py',          '{SDIR}',  node_pid('py-sim'),   restart(),  None,    ['sim.py -n barracuda\'']),
    Process('persist',         '{SDIR}',  node_pid('persist'),  restart(),  None,    ['persist.py -rs -f persist/sim']),
    Process('imgpipeline',     '/',       node_pid('img-pipe'), restart(),  None,    ['img-pipeline -n ai']),
]

def get_processes(args):
    return processes
