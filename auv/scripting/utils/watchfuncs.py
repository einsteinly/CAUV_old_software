# Standard Library
import os
import errno
import collections
import subprocess

# CAUV
import utils.zmqfuncs
from cauv.debug import debug, info, warning, error

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
            pid = ctrl.run_cmd("PID")["pid"]
            return pid;
        except utils.zmqfuncs.Timeout:
            return None
    return get_pid

def search_pid(program_name):
    def get_pid():
        # this could be improved...
        ps = subprocess.check_output(['ps', '-opid,command', '-A'])
        processes = ps.split('\n')
        for p in processes:
            if p.find(program_name) != -1:
                return int(p.strip().split()[0])
        return None
    return get_pid

def restart(n_times = 0):
    vals = {'restarts' : 0}
    def death_callback(proc):
        vals['restarts'] += 1
        if n_times == 0 or vals['restarts'] <= n_times:
            proc.restart()
        else: 
            warning("Process {} died too many times. Not restarting".format(proc.name))
    return death_callback

def restart_also(*others):
    def restart_callback(proc):
        for other in others:
            proc.watcher.restart(other)
    return restart_callback

def panic(proc):
    raise RuntimeError("Important process {} crashed! panic!".format(proc.name))

def ignore(proc):
    pass

def no_deps(proc):
    return True

def depends_on(*others):
    def prereq(proc):
        can_run = True
        for other in others:
            if not proc.watcher.is_running(other):
                can_run = False
                try:
                    info("Starting {} for {}".format(other, proc.p.name))
                    proc.watcher.start(other)
                except KeyError:
                    error("Dependency for {}, {}, doesn't exist!".format(proc.p.name, other))
                    proc.stop([])
                    return False
        return can_run
    return prereq

class Process(object):
    def __init__(self, name, cmd, get_pid = None, death = ignore, prereq = no_deps, autostart = False, restart = lambda x: None):
        self.name = name
        self.cmd = cmd
        if get_pid is None:
            self.get_pid = search_pid(cmd[0])
        else:
            self.get_pid = get_pid
        self.death_callback = death
        self.prereq = prereq
        self.autostart = autostart
        self.restart_callback = restart
