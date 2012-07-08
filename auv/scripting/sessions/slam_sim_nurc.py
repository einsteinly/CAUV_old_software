from utils.watchfuncs import *

processes = [
    Process('control',     '{BDIR}', node_pid('Control'),  restart(), None,
        ['{BDIR}/control{D} -N']
    ),
    Process('sim.py',      '{SDIR}', node_pid('py-sim'),   panic,     None,
        ["sh -c 'sleep 2 && {SDIR}/run.sh {SDIR}/simulator/sim.py'"]
    ),
    Process('img-pipeline','{BDIR}', node_pid('img-pipe'), panic,     None,
        ['{BDIR}/img-pipeline']
    ),
    Process('savepl.py',   '{SDIR}', node_pid('py-play'),  panic,      None,
        ["sh -c 'sleep 2 && {SDIR}/run.sh {SDIR}/savepl.py -f {SDIR}/pipelines/slam.pipe load'"]
    ),
    Process('persist',     '{SDIR}', node_pid('persist'),  restart(), None,
        ['{SDIR}/persist.py -rs']
    ),
    Process('fake-gemini', '{BDIR}', node_pid('FakeGem'),  panic,     None,
        ["sh -c 'sleep 6 && {BDIR}/fake_gemini -e {SDIR}/tests/resources/comp_arena_2d.png -s 0.038 -x -700 -y -400'"]
    ),
]

def get_processes(args):
    return processes

