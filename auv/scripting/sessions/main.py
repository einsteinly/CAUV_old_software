#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from utils.watchfuncs import *
import utils.dirs

import socket
import networking

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
    Process('camera_server', ['camera_server']),
    Process('pipeline', ['img-pipeline', '-n', 'ai'], node_pid('img-pipe'),
                death = restart(), prereq = depends_on('camera_server')),
    Process('p-resort', ['penultimate-resort.py']),
    Process('setup', ['true'],
                death = ignore, prereq = depends_on('pipeline', 'sonar', 'control', 'daemon-man', 'p-resort')),
    Process('task_manager', ['task_manager.py'], node_pid('task_manager'), death = restart(), prereq = depends_on('pipeline', 'location_manager'))
    Process('location_manager', ['location_manager.py', '-b', '17', '-f', 'real_sonar_walls', '-d', '50'], node_pid('location_manager'), death = restart(), prereq = depends_on('pipeline')),
    Process('temp_monitor', ['temperaturemonitor.py'], node_pid('temperaturemonitor.py'), death = restart(4))
]

laptop_processes = [
    Process('gamepad', ['gamepad_server.py'], autostart=True),
    Process('gui', ['gui'], autostart=True)
]

def get_arguments(group):
    networking.get_arguments(group)

def get_processes(args):
    p = networking.get_processes(args)
    if args.hw == 'laptop':
        p += laptop_processes
    elif args.hw == 'barracuda':
        p += barracuda_processes
    return p
