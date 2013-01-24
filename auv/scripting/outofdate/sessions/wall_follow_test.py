#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from utils.watchfuncs import *

processes = [
    #Process('vehicle_daemon',  '/',       zmq_daemon_pid(),     panic,      None,    ['{BDIR}/vehicle_daemon{D} -s']),
    Process('control',         '/',       node_pid('Control'),  restart(),  'root',  ['{BDIR}/control{D} -N']),
    Process('sim.py',          '{SDIR}',  node_pid('py-sim'),   restart(),  None,    ["sh -c 'PYTHONPATH=$PYTHONPATH:{SDIR} python2.7 {SDIR}/simulator/sim.py'"]),
    Process('persist',         '{SDIR}',  node_pid('persist'),  restart(),  None,    ["sh -c 'PYTHONPATH=$PYTHONPATH:{SDIR} python2.7 {SDIR}/persist.py -rs'"]),
    Process('imgpipeline',     '/',       node_pid('img-pipe'), restart(),  None,    ['{BDIR}/img-pipeline -n ai']),
    Process('fake-gemini',     '{BDIR}',   node_pid('FakeGem'), panic,      None,
        ["sh -c 'sleep 3 && {BDIR}/fake_gemini -e {SDIR}/tests/resources/comp_arena_2d.png -s 0.03 -x -400 -y 400'"]
    ),
]

def get_processes(args):
    return processes
