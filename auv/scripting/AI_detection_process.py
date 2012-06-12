#!/usr/bin/env python2.7
from cauv.debug import debug, warning, error, info
from cauv import messaging

import threading, Queue
import time
import traceback

from AI_classes import aiProcess, external_function, aiDetectorOptions

MAX_WAITING_TIME = 1

class detectionControl(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'detector_control')
        self.modules = {}
        self.running_detectors = {}
        self.requests = Queue.Queue()
        self.enable_flag = threading.Event()
        self.enable_flag.set()
    @external_function
    def start(self, detector_id, detector_file, options={}):
        self.requests.put(('start', detector_id, detector_file, options))
    @external_function
    def stop(self, detector_id):
        self.requests.put(('stop', detector_id))
    @external_function
    def set_options(self, detector_id, options):
        self.requests.put(('set_options', detector_id, options))
    @external_function
    def disable(self):
        self.enable_flag.clear()
    @external_function
    def enable(self):
        info("Re enabling detectors")
        self.enable_flag.set()
    def run(self):
        while True:
            if not self.enable_flag.is_set():
                for running_detector in self.running_detectors.values():
                    running_detector.die()
                self.running_detectors = {}
                info('Detector process disabled')
                self.enable_flag.wait(2)
                continue
            #update running detectors from requests (has to be done here as list of running detectors is constantly in use by this process)
            while True:
                try:
                    command = self.requests.get(block = True, timeout = MAX_WAITING_TIME)
                except Queue.Empty:
                    break
                if command[0] == 'start':
                    detector_id, detector_file, options = command[1:4]
                    if detector_id in self.running_detectors:
                        debug("Detector id %s is already running." %(detector_id))
                    else:
                        #make sure that the detection file has been imported
                        if not (detector_file in self.modules):
                            #interesting behaviour of __import__ here
                            try:
                                self.modules[detector_file] = __import__('detector_library.'+detector_file, fromlist=['detector_library'])
                            except Exception:
                                error('Could not import detector %s.' %(detector_file,))
                                traceback.print_exc()
                                continue
                        #load and set options
                        try:
                            opts = self.modules[detector_file].detectorOptions(options)
                        except Exception:
                            error('Could not initialise detector %s options, trying with no options.' %(detector_file,))
                            traceback.print_exc()
                            opts = aiDetectorOptions({})
                        #start detector
                        try:
                            self.running_detectors[detector_id] = self.modules[detector_file].detector(self.node, opts)
                        except Exception:
                            error('Could not initialise detector %s.' %(detector_file,))
                            traceback.print_exc()
                            continue
                        info("Started detector class, id %s, %s." %(detector_file, detector_id))
                elif command[0] == 'stop':
                    detector_id = command[1]
                    try:
                        self.running_detectors[detector_id].die()
                        self.running_detectors.pop(detector_id)
                    except KeyError:
                        debug(detector_id+" does not exist, so cannot be stopped")
                    except Exception as e:
                        error('Could not kill detector %s.' %(detector_id,))
                        traceback.print_exc()
                    else:
                        info("Stopped detector %s." %(detector_id))
                elif command[0] == 'set_options':
                    detector_id, options = command[1], command[2]
                    try:
                        for key, value in options.iteritems():
                            self.running_detectors[detector_id].set_option(key, value)
                    except KeyError:
                        error("Detector %s doesn't exist, options can't be changed" %(detector_id))
            #send status
            self.ai.task_manager.on_list_of_detectors(self.running_detectors.keys())
            pipelines = []
            #run detection
            for detector_id, detector in self.running_detectors.iteritems():
                pipelines.extend(detector._pipelines)
                #since each processing could take a while, and disabling needs to be pretty fast, check here
                if not self.enable_flag.is_set():
                    break
                try:
                    detector.process()
                except Exception:
                    error('Exception while running %s.' %(detector_id,))
                    traceback.print_exc()
                if detector.detected:
                    self.ai.task_manager.notify_detector(detector_id, True)
                self.node.send(messaging.ConditionStateMessage(detector_id, detector.options.get_options_as_params(),
                               detector.get_debug_values(), detector._pipelines))
            self.ai.pl_manager.set_detector_pls(pipelines)

if __name__ == '__main__':
    try:
        dc = detectionControl()
        dc.run()
    finally:
        dc.die()
