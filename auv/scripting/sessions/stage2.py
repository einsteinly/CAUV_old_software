import socket

from utils.watchfuncs import *
import stage1


if socket.gethostname() == 'barracuda-seco':
    # processes running on seco only
    processes = [
        Process('imgpipe default', '/',         node_pid('img-pipe'),         restart(),    None,    ['{BDIR}/img-pipeline{D}']),
        Process('camera server',   '/',         pid_search('camera_server'),  restart(3),   None,    ['{BDIR}/camera_server{D}']),
        Process('gpsd',            '{SDIR}',    node_pid('py-gps'),           restart(3),   None,    ['{SDIR}/gpsd.py']),
        Process('gemini_node 17',  '/',         node_pid('Gemini'),           restart(),    None,    ['{BDIR}/gemini_node{D} 17']),
    ]
elif socket.gethostname() == 'barracuda-arm':
    # processes running on the panda board only 
    processes = [
        #Process(), ??
    ]

processes = stage1.get_processes() + processes

def get_processes(args):
    return processes


