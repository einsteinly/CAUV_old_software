import os
import errno
import collections
import utils.zmqfuncs
from cauv.debug import debug, info, warning, error

Process = collections.namedtuple("Process",['name', 'work_dir', 'pid_func', 'death_callback', 'user', 'cmds'])

def node_pid(node_name, vehicle_dir = utils.zmqfuncs.get_vehicle_dir()):
    def get_pid():
        if not os.path.exists(vehicle_dir):
            return None
        for socket_filename in os.listdir(vehicle_dir):
            try:
                name, pid = socket_filename.rsplit('.',2)
                if node_name == name:
                    try:
                        os.kill(int(pid),0)
                    except OSError as e:
                        if e.errno == errno.ESRCH:
                            continue
                    return int(pid)
            except ValueError:
                pass
        return None
    return get_pid

def zmq_daemon_pid(ipc_dir = None, vehicle_name = None):
    def get_pid():
        ctrl = utils.zmqfuncs.DaemonControl(vehicle_name, ipc_dir)
        try:
            return(ctrl.run_cmd("PID")["pid"])
        except utils.zmqfuncs.Timeout:
            return None
    return get_pid

def restart(n_times = 0):
    vals = {'restarts' : 0}
    def death_callback(proc):
        vals['restarts'] += 1
        if n_times == 0 or vals['restarts'] <= n_times:
            return True
        else: 
            warning("Process {} died too many times. Not restarting".format(proc.name))
            return False
    return death_callback

def panic(proc):
    raise RuntimeError("Important process {} crashed! panic!".format(proc.name))

