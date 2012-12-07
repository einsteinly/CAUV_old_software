from utils.watchfuncs import *
import ai_procs

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
    
]

def get_arguments(group):
    group.add_argument('--map', choices = ['rivermap', 'saucemap'], help="Hardware to run on",
                        default = 'saucemap')
def get_processes(args):
    if args.map == 'rivermap':
        processes.append(Process('sim', ['sim', '-f', utils.dirs.config_dir('environment/rivermap.obj'), '-g'], node_pid('sim'), prereq = depends_on('control', 'pipeline')))
    elif args.map == 'saucemap':
        processes.append(Process('sim', ['sim', '-f', utils.dirs.config_dir('environment/saucemap.obj'), '-g'], node_pid('sim'), prereq = depends_on('control', 'pipeline')))
    processes.extend(ai_procs.get_processes(args))
    return processes
