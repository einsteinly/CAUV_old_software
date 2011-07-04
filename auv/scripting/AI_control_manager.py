import cauv.node
import cauv.control as control
from cauv.debug import debug, warning, error, info

import time
import threading
import optparse

from AI_classes import aiProcess, external_function

#TODO basically the actual functionality of conrol, the ability to stop the sub, block script_ids etc

class auvControl(aiProcess):
    def __init__(self, **kwargs):
        aiProcess.__init__(self, 'auv_control')
        self.auv = control.AUV(self.node)
        self.external_functions = []
        self.script_lock = threading.Lock()
        self.current_task_id = None
        self.enabled = threading.Event()
        if 'disable_control' in kwargs:
            if not kwargs['disable_control']:
                self.enabled.set()
        self.limit_lock = threading.Lock()
        self.depth_limit = None
    @external_function
    def auv_command(self, task_id, command, *args, **kwargs):
        #__getattr__ was more trouble than its worth. since this is abstracted by fakeAUV, doesn't matter to much
        #TODO make it possible to filter by script id. script id should match task_managers record as well
        #Might need to move parts of control here/take a smaller version of control that doesn't have waiting commands (eg depth and wait)
        #note, we don't care about errors here, cos they'l be caught by the message handler.
        #Also the message handler will tell us which message from who caused the error
        with self.script_lock:
            if self.enabled.is_set() and self.current_task_id == task_id:
                getattr(self.auv, command)(*args, **kwargs)
    @external_function
    def set_task_id(self, task_id):
        with self.script_lock:
            self.current_task_id = task_id
    @external_function
    def enable(self):
        self.enabled.set()
    @external_function
    def disable(self):
        self.enabled.clear()
        self.auv.stop()
    @external_function
    def stop(self):
        #if the sub keeps turning to far, it might be an idea instead of calling stop which disables auto pilots to set them to the current value
        self.auv.prop(0)
        self.auv.hbow(0)
        self.auv.vbow(0)
        self.auv.hstern(0)
        self.auv.vstern(0)
        self.auv.bearing(self.auv.current_bearing)
        self.auv.pitch(0)
        self.auv.depth(self.auv.current_depth)
    @external_function
    def signal(self, value):
        info('signalling %d' %(value,))
        self.auv.forwardlights(0)
        time.sleep(2)
        for b in bin(value)[2:]:
            self.auv.forwardlights(255)
            if b == '1':
                time.sleep(1)
            else:
                time.sleep(0.5)
            self.auv.forwardlights(0)
            time.sleep(0.5)
    @external_function
    def depth(self, value):
        with self.limit_lock:
            if self.depth_limit and self.depth_limit<value:
                self.auv.depth(self.depth_limit)
                getattr(self.ai, self.current_task_id).depthOverridden()
                return
        self.auv.depth(value)
    @external_function
    def limit_depth(value):
        with self.limit_lock:
            self.depth_limit = value
    def run(self):
        while True:
            time.sleep(10)
            info("auv_control still alive")

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-d', '--disable_control', dest='disable_control', default=False,
                 action='store_true', help="stop AI script from controlling the sub")
    opts, args = p.parse_args()
    try:
        ac = auvControl()
        ac.run()
    finally:
        ac.die()
