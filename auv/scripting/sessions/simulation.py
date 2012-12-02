from utils.watchfuncs import *

processes = [
    Process('control', ['control', '-N'],
                node_pid('Control'), death = restart(), prereq = depends_on('py-sim', 'persist')),
    Process('py-sim', ['sim.py', '-n', 'barracuda'],
                node_pid('py-sim'), death = restart()),
    Process('persist',    ['persist.py', '--silent', '--restore', '--persistence-dir',
                            utils.dirs.config_dir('persist/barracuda')],
                node_pid('persist'), death = restart())
]

def get_processes(args):
    return processes
