#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from utils.watchfuncs import *

processes = [
    Process('control', ['control', '-N'],
                node_pid('Control'), death = restart(), prereq = depends_on('py-sim', 'persist')),
    Process('py-sim', ['sim.py', '-n', 'barracuda'],
                       node_pid('py-sim'), death = restart()),
    Process('persist', ['persist.py', '--silent', '--restore', '--persistence-dir',
                        utils.dirs.config_dir('persist/barracuda')],
            node_pid('persist'), death = restart()),
    Process('camera_server', ['camera_server']),
    Process('pipeline', ['img-pipeline', '-n', 'ai'], node_pid('img-pipe'), death = restart(), prereq = depends_on('camera_server')),
    Process('gamepad', ['gamepad_server.py'], node_pid('py-gpmd')),
    Process('gui', ['gui'], node_pid('CauvGui')),

    Process('camera_server', ['camera_server']),
    Process('p-resort', ['penultimate-resort.py']),
    Process('setup', ['true'],
                death = ignore, prereq = depends_on('pipeline', 'sonar', 'control', 'daemon-man', 'p-resort')),
    Process('task_manager', ['task_manager.py'], node_pid('task_manager'), death = restart(), prereq = depends_on('pipeline', 'location_manager')),
    Process('location_manager', ['location_manager.py'], node_pid('location_manager'), death = restart()),
]

def get_arguments(group):
    group.add_argument('--map', choices = ['rivermap', 'saucemap'], help="Hardware to run on",
                        default = 'saucemap')
def get_processes(args):
    if args.map == 'rivermap':
        processes.append(Process('sim', ['sim', '-f', utils.dirs.config_dir('environment/rivermap.obj'), '-g'], node_pid('sim'), prereq = depends_on('control', 'pipeline')))
    elif args.map == 'saucemap':
        processes.append(Process('sim', ['sim', '-f', utils.dirs.config_dir('environment/saucemap.obj'), '-g'], node_pid('sim'), prereq = depends_on('control', 'pipeline')))
    return processes
