from utils.watchfuncs import *
import utils.dirs

import socket
import networking
import ai_procs

barracuda_processes = [
    Process('persist',    ['persist.py', '--silent', '--restore', '--persistence-dir',
                            utils.dirs.config_dir('persist/barracuda')],
                node_pid('persist'), death = ignore),
    Process('control', ['control', '--sbg', '/tmp/ttyV1', '--port', '/tmp/ttyVCAN0', '--imu', 'sbg'],
                node_pid('Control'), prereq = depends_on('mcb_bridge', 'persist')),
    Process('mcb_bridge', ['mcb_bridge', '--port', '/dev/ttyS0', '--prefix', '/tmp/ttyV'],
                death = restart(), restart = restart_also('control')),
    Process('sonar', ['gemini_node', '--sonar_id', '17'], # 17 == serial number of gemini sonar
                death = restart()),
    Process('camera_setup', ['setup_cameras.sh']),
    Process('camera_server', ['camera_server']),
    Process('pipeline', ['img-pipeline'], node_pid('img-pipe'),
                death = restart(), prereq = depends_on('camera_server', 'camera_setup')),
    Process('p-resort', ['penultimate-resort.py']),
    Process('setup', ['true'],
                death = ignore, prereq = depends_on('pipeline', 'sonar', 'control', 'daemon-man', 'p-resort')),
]

laptop_processes = [
    Process('gamepad', ['gamepad_server.py']),
    Process('gui', ['gui'])
]

def get_arguments(group):
    group.add_argument('--hw', choices = ['sim', 'barracuda'], help="Hardware to run on",
                        default = 'laptop' if socket.gethostname().find('barracuda') == -1 else 'barracuda')

def get_processes(args):
    p = networking.get_processes(args)
    if args.hw == 'laptop':
        p += laptop_processes
    elif args.hw == 'barracuda':
        p += barracuda_processes
        p += ai_procs.get_processes(args)
    return p
