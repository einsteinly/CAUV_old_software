import os
import json
import zmq
from cauv.debug import debug, info, warning, error

def get_vehicle_dir(ipc_dir = None, vehicle_name = None):
    if ipc_dir is None:
        ipc_dir = os.getenv('CAUV_IPC_DIR')
    if ipc_dir is None:
        ipc_dir = '/tmp/cauv'
    if vehicle_name is None:
        vehicle_name = os.getenv('CAUV_VEHICLE_NAME')
    if vehicle_name is None:
        vehicle_name = 'red_herring'
    return os.path.join(ipc_dir, vehicle_name)

class DaemonControl:
    def __init__(self, vehicle_name = None, ipc_dir = None):
        self.zmq_context = zmq.Context(1)
        daemon_control_str = 'ipc://{}/daemon/control'.format(get_vehicle_dir(ipc_dir, vehicle_name))
        self.vehicle = vehicle_name
        debug("Connecting to daemon via {}".format(daemon_control_str))
        self.ctrl_s = zmq.Socket(self.zmq_context, zmq.REQ)
        self.ctrl_s.connect(daemon_control_str)

    def run_cmd(self, cmd, strip_code = True):
        self.ctrl_s.send(cmd)
        ret = json.loads(self.ctrl_s.recv())
        if not ret['success']:
            error("Error running daemon command {} : {}".format(cmd, ret['error']))
        if strip_code:
            del ret['success']
        return ret;
