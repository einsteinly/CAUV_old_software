from utils.watchfuncs import *
import utils.dirs

import socket

processes = [
    Process('daemon-man', ['daemon_man.py'], 
                node_pid('daemon-man'), death = restart(), prereq = depends_on('heimdall')),
    Process('heimdall',   ['heimdall', '-s'],
                zmq_daemon_pid(), death = restart(), restart = restart_also('daemon-man')),
    Process('persist',    ['persist.py', '--silent', '--restore', '--persistence-dir',
                            utils.dirs.config_dir('persist/barracuda')],
                node_pid('persist'), death = ignore)
]

sim_processes = [
    Process('control', ['control', '-N'],
                node_pid('Control'), death = restart(), prereq = depends_on('py-sim', 'persist')),
    Process('py-sim', ['sim.py', '-n', 'barracuda'],
                node_pid('py-sim'), death = restart()),
]

barracuda_processes = [
    Process('control', ['control', '--sbg', '/tmp/ttyV1', '--port', '/dev/ttyVCAN0', '--imu', 'sbg'],
                node_pid('Control'), prereq = depends_on('mcb_bridge', 'persist')),
    Process('mcb_bridge', ['mcb_bridge', '--port', '/dev/ttyS0', '--prefix', '/tmp/ttyV'],
                death = restart(), restart = restart_also('control')),
    Process('sonar', ['gemini_node', '--sonar_id', '17'], # dunno where 17 comes from.
                death = restart())
]

def get_arguments(group):
    group.add_argument('--hw', choices = ['sim', 'barracuda'], help="Hardware to run on",
                        default = 'sim' if socket.gethostname().find('barracuda') == -1 else 'barracuda')

def get_processes(args):
    p = processes
    if args.hw == 'sim':
        p += sim_processes
    elif args.hw == 'barracuda':
        p += barracuda_processes
    print(p)
    return p
