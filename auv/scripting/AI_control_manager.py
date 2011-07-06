import cauv.node
from cauv import control, sonar
from cauv.debug import debug, warning, error, info

import time
import threading, Queue
import optparse

from AI_classes import aiProcess, external_function

#TODO basically the actual functionality of conrol, the ability to stop the sub, block script_ids etc

class auvControl(aiProcess):
    def __init__(self, **kwargs):
        aiProcess.__init__(self, 'auv_control')
        self.auv = control.AUV(self.node)
        self.sonar = sonar.Sonar(self.node)
        self.external_functions = []
        self.current_task_id = None
        self.enabled = threading.Event()
        if 'disable_control' in kwargs:
            if not kwargs['disable_control']:
                self.enabled.set()
        self.depth_limit = None
        self.signal_msgs = Queue.Queue(5)
        self._register()
    @external_function
    def auv_command(self, task_id, command, *args, **kwargs):
        #__getattr__ was more trouble than its worth. since this is abstracted by fakeAUV, doesn't matter to much
        #TODO make it possible to filter by script id. script id should match task_managers record as well
        #Might need to move parts of control here/take a smaller version of control that doesn't have waiting commands (eg depth and wait)
        #note, we don't care about errors here, cos they'l be caught by the message handler.
        #Also the message handler will tell us which message from who caused the error
        debug('auvControl::auv_command(self, task_id=%s, cmd=%s, args=%s, kwargs=%s)' % (task_id, command, args, kwargs), 5)
        if self.enabled.is_set(): #and self.current_task_id == task_id:
            debug('Will call %s(*args, **kwargs)' % (getattr(self.auv, command)), 5)
            getattr(self.auv, command)(*args, **kwargs)
    @external_function
    def sonar_command(self, task_id, command, *args, **kwargs):
        if self.enabled.is_set() and self.current_task_id == task_id:
            getattr(self.sonar, command)(*args, **kwargs)
    @external_function
    def set_task_id(self, task_id):
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
        if self.auv.bearing != None:
            self.auv.bearing(self.auv.current_bearing)
        self.auv.pitch(0)
        if self.auv.depth != None:
            self.auv.depth(self.auv.current_depth)
    @external_function
    def lights_off(self):
        self.auv.downlights(0)
        self.auv.forwardlights(0)
    @external_function
    def signal(self, value):
        try:
            self.signal_msgs.put(value)
        except Queue.Full:
            error('Signalling queue is full, dropping request for signal')
    @external_function
    def depth(self, value):
        if self.depth_limit and self.depth_limit<value:
            self.auv.depth(self.depth_limit)
            getattr(self.ai, self.current_task_id).depthOverridden()
        else:
            self.auv.depth(value)
    @external_function
    def limit_depth(self, value):
        self.depth_limit = value
    def run(self):
        self.auv.forwardlights(0)
        while True:
            try:
                msg = self.signal_msgs.get(block = True, timeout = 5)
            except Queue.Empty:
                continue
            self.auv.forwardlights(0)
            time.sleep(2)
            for c in str(msg):
                if not c in morsetab:
                    continue
                for c2 in morsetab[c]:
                    self.auv.forwardlights(255)
                    if c2 == '-':
                        time.sleep(1)
                    else:
                        time.sleep(0.5)
                    self.auv.forwardlights(0)
                    time.sleep(0.5)
                time.sleep(1)

#from Demos/scripts/morse.py
morsetab = {
        'A': '.-',              'a': '.-',
        'B': '-...',            'b': '-...',
        'C': '-.-.',            'c': '-.-.',
        'D': '-..',             'd': '-..',
        'E': '.',               'e': '.',
        'F': '..-.',            'f': '..-.',
        'G': '--.',             'g': '--.',
        'H': '....',            'h': '....',
        'I': '..',              'i': '..',
        'J': '.---',            'j': '.---',
        'K': '-.-',             'k': '-.-',
        'L': '.-..',            'l': '.-..',
        'M': '--',              'm': '--',
        'N': '-.',              'n': '-.',
        'O': '---',             'o': '---',
        'P': '.--.',            'p': '.--.',
        'Q': '--.-',            'q': '--.-',
        'R': '.-.',             'r': '.-.',
        'S': '...',             's': '...',
        'T': '-',               't': '-',
        'U': '..-',             'u': '..-',
        'V': '...-',            'v': '...-',
        'W': '.--',             'w': '.--',
        'X': '-..-',            'x': '-..-',
        'Y': '-.--',            'y': '-.--',
        'Z': '--..',            'z': '--..',
        '0': '-----',           ',': '--..--',
        '1': '.----',           '.': '.-.-.-',
        '2': '..---',           '?': '..--..',
        '3': '...--',           ';': '-.-.-.',
        '4': '....-',           ':': '---...',
        '5': '.....',           "'": '.----.',
        '6': '-....',           '-': '-....-',
        '7': '--...',           '/': '-..-.',
        '8': '---..',           '(': '-.--.-',
        '9': '----.',           ')': '-.--.-',
        ' ': ' ',               '_': '..--.-',
}

if __name__ == '__main__':
    p = optparse.OptionParser()
    p.add_option('-d', '--disable_control', dest='disable_control', default=False,
                 action='store_true', help="stop AI script from controlling the sub")
    opts, args = p.parse_args()
    ac = auvControl()
    try:
        ac.run()
    finally:
        ac.die()
