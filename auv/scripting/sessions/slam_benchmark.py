from utils.watchfuncs import *

processes = [
    Process('control',        '{BDIR}', node_pid('Control'),  restart(), None,
        ['{BDIR}/auv/bin/control{D} -N']
    ),
    Process('sim.py',         '{SDIR}', node_pid('py-sim'),   panic,     None,
        ['sh -c \'PYTHONPATH=$PYTHONPATH:{SDIR} python2.7 {SDIR}/simulator/sim.py\'']
    ),
    Process('persist',        '{SDIR}', node_pid('persist'),  restart(), None,
        ['{SDIR}/persist.py -rs']
    ),
    #Process('img-pipeline',   '{BDIR}', node_pid('img-pipe'), panic,     None,
    #    ['{BDIR}/auv/bin/img-pipeline']
    #),
    Process('fake-gemini',    '{BDIR}', node_pid('FakeGem'),  panic,     None,
        ['{BDIR}/auv/bin/fake_gemini -e /Users/james/Documents/IIB/Project/test_environment_mid.png -s 0.05 -x -100 -y -100']
    ),
    Process('slam-benchmark', '{SDIR}', node_pid('py-slamb'), panic,     None,
        ['{SDIR}/tests/slam_benchmark.py']
    ),
]

def get_processes(args):
    return processes
