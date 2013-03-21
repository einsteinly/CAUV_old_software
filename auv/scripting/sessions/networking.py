#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from utils.watchfuncs import *

def get_arguments(group):
    group.add_argument('--hw', choices = ['sim', 'barracuda'], help="Hardware to run on",
                        default = 'laptop' if socket.gethostname().find('barracuda') == -1 else 'barracuda')

def get_processes(args):
    if args.hw == 'laptop':
        peer = '10.0.0.2'
    elif args.hw == 'barracuda':
        peer = '10.0.0.3'
    processes = [
        Process('daemon-man', ['daemon_man.py', '--peer', peer], 
                    node_pid('daemon-man'), death = restart(), prereq = depends_on('heimdall')),
        Process('heimdall',   ['heimdall', '-s'],
                    zmq_daemon_pid(), death = restart(), restart = restart_also('daemon-man')),
    ]
    return processes
