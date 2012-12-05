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
    Process('pipeline', ['img-pipeline'], node_pid('img-pipe'), death = restart(), prereq = depends_on('camera_server')),
    Process('sim', ['sim', '-f', utils.dirs.config_dir('environment/saucemap.obj')], node_pid('sim'), prereq = depends_on('py-sim')),
    Process('gamepad', ['gamepad_server.py'], node_pid('py-gpmd')),
    Process('gui', ['gui'], node_pid('CauvGui')),
]

def get_processes(args):
    processes.extend(ai_procs.get_processes(args))
    return processes
