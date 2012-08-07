from utils.watchfuncs import *

processes = [
    Process('control',     '{SDIR}', node_pid('Control'),  restart(), None,
        ['cauv-control -N']
    ),
    Process('sim.py',      '{SDIR}', node_pid('py-sim'),   panic,     None,
        ["sh -c 'sleep 2 && {SDIR}/run.sh {SDIR}/simulator/sim.py'"]
    ),
    Process('img-pipeline','{SDIR}', node_pid('img-pipe'), panic,     None,
        ['cauv-img-pipeline']
    ),
    # really, want a restart-if-img-pipeline-dies option here... or the loaded
    # pipeline to be persistent state
    Process('savepl.py',   '{SDIR}', node_pid('py-play'),  ignore,    None,
        ["sh -c 'sleep 2 && {SDIR}/run.sh {SDIR}/savepl.py -f {SDIR}/pipelines/slam.pipe load -n default/demo'"]
    ),
    Process('persist',     '{SDIR}', node_pid('persist'),  restart(), None,
        ['{SDIR}/persist.py -rsf {SDIR}/persist/demo']
    ),
    #Process('fake-gemini', '{SDIR}', node_pid('FakeGem'),  panic,     None,
    #    ["sh -c 'sleep 6 && cauv-fake_gemini -e {SDIR}/tests/resources/comp_arena_2d.png -s 0.038 -x -700 -y -400'"]
    #),
    Process('gui',         '{SDIR}', node_pid('CauvGui'), restart(), None,
        ['cauv-gui']
    ),
]

def get_processes(args):
    return processes
