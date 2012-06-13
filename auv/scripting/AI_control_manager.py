#!/usr/bin/env python2.7
import cauv.node
from cauv import control, sonar
from cauv.debug import debug, warning, error, info

import time
import threading, Queue
import argparse

from AI_classes import aiProcess, external_function, force_calling_process

#TODO basically the actual functionality of control, the ability to stop the sub, block script_ids etc

control_listen_to = ['prop', 'strafe', ]

class slightlyModifiedAUV(control.AUV):
    def __init__(self, node):
        control.AUV.__init__(self, node)
        self.depth_limit = None
        self.prop_limit = None
    def depth(self, value):
        if self.depth_limit and self.depth_limit<value:
            control.AUV.depth(self, self.depth_limit)
            #getattr(self.ai, self.current_task_id).depthOverridden()
        else:
            control.AUV.depth(self, value)
    def prop(self, value):
        if self.prop_limit and self.prop_limit<value:
            control.AUV.prop(self, self.prop_limit)
            #getattr(self.ai, self.current_task_id).propOverridden()
        else:
            control.AUV.prop(self, value)
    

class auvControl(aiProcess):
    def __init__(self, **kwargs):
        aiProcess.__init__(self, 'auv_control')
        self.auv = slightlyModifiedAUV(self.node)
        self.auv.depth_disabled = False
        self.sonar = sonar.Sonar(self.node)
        self.active_tasks = {}
        self.default_task = None
        self.waiting_for_control = {}
        self.current_task = None
        self.enabled = threading.Event()
        if 'disable_control' in kwargs:
            if not kwargs['disable_control']:
                self.enabled.set()
        else: self.enabled.set()
        self.depth_limit = None
        self.signal_msgs = Queue.Queue(5)
        self.processing_queue = Queue.Queue()
        #values set by the current script
        self._control_state = {}
        #values set by the default script
        self._control_state_default = {}
        self._sonar_state_default = {}
        
    #SCRIPT COMMANDS
    @external_function
    @force_calling_process
    def auv_command(self, command, *args, **kwargs):
        task_id = kwargs.pop('calling_process')
        debug('auvControl::auv_command(self, calling_process=%s, cmd=%s, args=%s, kwargs=%s)' % (task_id, command, args, kwargs), 5)
        if self.enabled.is_set():
            if self.current_task == task_id:
                debug('Will call %s(*args, **kwargs)' % (getattr(self.auv, command)), 5)
                getattr(self.auv, command)(*args, **kwargs)
                self._control_state[command] = (args, kwargs)
            else:
                warning('Script %s tried to move auv when script %s was in control \
                (scripts waiting for control were %s, scripts known to be running were %s)'
                %(task_id, self.current_task, self.waiting_for_control, self.active_tasks))
        else:
            debug('Function not called as paused or disabled.', 5)
    @external_function
    @force_calling_process
    def sonar_command(self, command, *args, **kwargs):
        task_id = kwargs.pop('calling_process')
        if self.enabled.is_set() and self.current_task == task_id:
            getattr(self.sonar, command)(*args, **kwargs)
            
    @external_function
    @force_calling_process
    def request_control(self, timeout=None, *args, **kwargs):
        self.processing_queue.put(('request_control_local',[kwargs['calling_process'], timeout],{}))
    def request_control_local(self, calling_process, timeout=None):
        try:
            self.waiting_for_control[calling_process] = (self.active_tasks[calling_process], timeout+time.time() if timeout else None)
        except KeyError:
            warning('Script %s wanted to take control of auv when not in list of allowed scripts \
            (scripts waiting for control were %s, scripts known to be running were %s)'
            %(calling_process, self.waiting_for_control, self.active_tasks))
            return
        if timeout:
            debug('Script %s has started waiting for control, with a %d timeout' %(calling_process, timeout))
        else:
            debug('Script %s has started waiting for control, with no timeout' %(calling_process))
    @external_function
    @force_calling_process
    def drop_control(self, *args, **kwargs):
        self.processing_queue.put(('drop_control_local',[kwargs['calling_process']],{}))
    def drop_control_local(self, task_id):
        self.waiting_for_control.pop(task_id, None)
        #need to tel script its been paused, else if it wants control in the future it will think it already has it.
        getattr(self.ai, task_id)._set_paused()
            
    #GENERAL FUNCTIONS (that could be called from anywhere)
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
        #check that depth autopilot has been set,
        if 'depth' in self._control_state and self._control_state['depth'] != None:
            self.auv.depth(self._control_state)
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
    
    #TASK MANAGER COMMANDS
    @external_function
    def set_current_task_id(self, task_id, priority):
        self.processing_queue.put(('set_current_task_id_local',[task_id, priority],{}))
    @external_function
    def add_additional_task_id(self, task_id, priority):
        self.processing_queue.put(('add_additional_task_id_local',[task_id, priority],{}))
    @external_function
    def remove_additional_task_id(self, task_id):
        self.processing_queue.put(('remove_additional_task_id_local',[task_id],{}))
        
    def set_current_task_id_local(self, task_id, priority):
        try:
            self.waiting_for_control.pop(self.default_task)
        except KeyError:
            #presumably no task in the list
            pass
        #if currently in control, need to stop
        if self.current_task == self.default_task:
            self.stop()
        else:
            #else need to clear default script state so new script gets fresh state
            #except for depth
            if 'depth' in self._control_state_default:
                self._control_state_default = {'depth': self._control_state_default['depth']}
            else:
                self._control_state_default = {}
        self.default_task = task_id
        #dont add to list if default set to none
        if task_id:
            self.waiting_for_control[task_id] = (priority, None)
    def add_additional_task_id_local(self, task_id, priority):
        self.active_tasks[task_id] = priority
    def remove_additional_task_id_local(self, task_id):
        self.active_tasks.pop(task_id)
        #try to remove from waiting for control list
        self.waiting_for_control.pop(task_id, None)
        getattr(self.ai, task_id)._set_paused()
        
    #OBSTACLE AVOIDANCE COMMANDS
    @external_function
    def enable(self):
        self.enabled.set()
    @external_function
    def disable(self):
        self.enabled.clear()
        self.auv.stop()
    @external_function
    def limit_depth(self, value):
        self.auv.depth_limit = value
    @external_function
    def limit_prop(self, value):
        self.auv.prop_limit = value
        try:
            if self._control_state['prop'][0][0]>value: #_control_state[function][args/kwargs]
                self.auv.prop(value)
        except KeyError:
            self.auv.prop(0)
        
    #signaling thread
    def signal_loop(self):
        self.auv.forwardlights(0)
        while True:
            try:
                msg = self.signal_msgs.get(block = False)
            except Queue.Empty:
                break
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
                
    #MAIN LOOP
    def reevaluate(self):
        #filter out requests that have timed out
        new_dict = {}
        for task_id, (priority, timeout) in self.waiting_for_control.iteritems():
            if timeout and timeout < time.time():
                getattr(self.ai, task_id)._set_paused()
                getattr(self.ai, task_id).control_timed_out()
                continue
            new_dict[task_id] = (priority, timeout)
        self.waiting_for_control = new_dict
        #find highest priority
        try:
            highest_priority_task = max(self.waiting_for_control.iteritems(), key=lambda x: x[1][0])[0]
        except ValueError:
            #list is empty
            highest_priority_task = None
        if self.current_task == highest_priority_task:
            #no change, so don't do anything
            return
        #need to: store current values, set highest priority as current task, make sure scripts know whether they are in control
        #atm, just store values for default script TODO decide on a better way of doing this (or even if storing values is desirable)
        if self.current_task == self.default_task:
            self._sonar_state_default = self.sonar.__dict__.copy()
            self._control_state_default = self._control_state.copy()
        #stop current script and restore values
        self.current_task = None
        self.stop()
        if highest_priority_task == self.default_task:
            for command, (args, kwargs) in self._control_state_default.items():
                getattr(self.auv, command)(*args, **kwargs)
            self._control_state = self._control_state_default
            #restore sonar state
            self.sonar.__dict__.update(self._sonar_state_default)
            self.sonar.update()
        self.current_task = highest_priority_task
        for task_id in self.waiting_for_control:
            if task_id != self.current_task:
                getattr(self.ai, task_id)._set_paused()
        if self.current_task:
            getattr(self.ai, self.current_task)._set_unpaused()
                    
    def run(self):
        signalling_thread = threading.Thread(target=self.signal_loop)
        signalling_thread.start()
        while True:
            if not self.signal_msgs.empty():
                if not signalling_thread.is_alive():
                    signalling_thread = threading.Thread(target=self.signal_loop)
                    signalling_thread.start()
            while True:
                try:
                    function, args, kwargs = self.processing_queue.get(block=True, timeout=1)
                except Queue.Empty:
                    break
                getattr(self, function)(*args,**kwargs)
                self.reevaluate()
    def die(self):
        self.disable()
        aiProcess.die(self)

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
    p = argparse.ArgumentParser()
    p.add_argument('-d', '--disable_control', dest='disable_control', default=False,
                 action='store_true', help="stop AI script from controlling the sub")
    opts, args = p.parse_known_args()
    ac = auvControl()
    try:
        ac.run()
    finally:
        ac.die()
