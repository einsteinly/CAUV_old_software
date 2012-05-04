import os
import errno
import collections
import util.zmqfuncs

Process = collections.namedtuple("Process",['name', 'work_dir', 'pid_func', 'importance', 'cmds'])

def node_pid(node_name, vehicle_dir = util.zmqfuncs.get_vehicle_dir()):
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

def zmq_daemon_pid(vehicle_dir = util.zmqfuncs.get_vehicle_dir()):
    import zmq
    def get_pid():
        c = zmq.Context()
        s = zmq.Socket(c, zmq.REQ)
        s.connect("ipc://{}/daemon/control".format(vehicle_dir))
        s.send("PID")
        s.setsockopt(zmq.LINGER,0)
        p = zmq.Poller()
        p.register(s, zmq.POLLIN)
        if len(p.poll(100)):
            pid = s.recv(copy=True, flags=zmq.NOBLOCK)
            return int(pid)
        return None
    return get_pid

