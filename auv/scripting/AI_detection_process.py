#!/usr/bin/env python2.7
from cauv.debug import debug, warning, error, info
from cauv import messaging

import threading
import traceback
import utils.event as event

from AI_classes import aiProcess, external_function, aiDetectorOptions

MAX_WAITING_TIME = 1

class detectionControl(aiProcess):
    def __init__(self):
        aiProcess.__init__(self, 'detector_control')
        self.modules = {}
        self.running_detectors = {}
        self.enable_flag = threading.Event()
        self.enable_flag.set()
        
        #start receiving messages
        self._register()

    @external_function
    @event.event_func
    def start(self, detector_id, detector_file, options={}):
        if detector_id in self.running_detectors:
            debug("Detector id %s is already running." %(detector_id))
            return
        #make sure that the detection file has been imported
        if not (detector_file in self.modules):
            #interesting behaviour of __import__ here
            try:
                self.modules[detector_file] = __import__('detector_library.'+detector_file, fromlist=['detector_library'])
            except Exception:
                error('Could not import detector %s.' %(detector_file,))
                error(traceback.format_exc().encode('ascii','ignore'))
                return
        #load and set options
        try:
            opts = self.modules[detector_file].detectorOptions(options)
        except Exception:
            error('Could not initialise detector %s options, trying with no options.' %(detector_file,))
            error(traceback.format_exc().encode('ascii', 'ignore'))
            opts = aiDetectorOptions({})
        #start detector
        try:
            self.running_detectors[detector_id] = self.modules[detector_file].detector(self.node, opts)
        except Exception:
            error('Could not initialise detector %s.' %(detector_file,))
            traceback.print_exc()
            return
        info("Started detector class, id %s, %s." %(detector_file, detector_id))
        self.detectors_changed()

    @external_function
    @event.event_func
    def stop(self, detector_id):
        try:
            self.running_detectors[detector_id].die()
            self.running_detectors.pop(detector_id)
        except KeyError:
            debug(detector_id+" does not exist, so cannot be stopped")
        except Exception as e:
            error('Could not kill detector %s.' %(detector_id,))
            error(traceback.format_exc().encode('ascii','ignore'))
        else:
            info("Stopped detector %s." %(detector_id))
        self.detectors_changed()

    @external_function
    @event.event_func
    def set_options(self, detector_id, options):
        try:
            for key, value in options.iteritems():
                self.running_detectors[detector_id].set_option(key, value)
        except KeyError:
            error("Detector %s doesn't exist, options can't be changed" %(detector_id))
        self.detectors_changed()

    @external_function
    def disable(self):
        self.enable_flag.clear()
        for detector in self.running_detectors.values():
            detector.die()
            self.running_detectors = {}
        self.detectors_changed()

    @external_function
    def enable(self):
        info("Re enabling detectors")
        self.enable_flag.set()
        self.detectors_changed()

    @event.repeat_event(1, True)
    def process_detectors(self):
        for detector_id, detector in self.running_detectors.iteritems():
            #since each processing could take a while, and disabling needs to be pretty fast, check here
            if not self.enable_flag.is_set():
                break
            try:
                detector.process()
            except Exception:
                error('Exception while running %s.' %(detector_id,))
                error(traceback.format_exc().encode('ascii','ignore'))
            if detector.detected != detector._detected_past:
                self.ai.task_manager.on_detector_state_change(detector_id, detector.detected)
                detector._detected_past = detector.detected
            self.node.send(messaging.ConditionStateMessage(detector_id, detector.options.get_options_as_params(),
                           detector.get_debug_values(), detector._pipelines))

    @event.repeat_event(5, True)
    def force_reeval(self):
        self.detectors_changed()
                           
    def detectors_changed(self):
        pipelines = []
        for detector in self.running_detectors.values():
            pipelines.extend(detector._pipelines)
            debug("{}".format(detector._pipelines), 3)
        self.ai.pl_manager.set_detector_pls(pipelines)
        self.ai.task_manager.on_list_of_detectors(self.running_detectors.keys())

if __name__ == '__main__':
    try:
        dc = detectionControl()
        dc.run()
    finally:
        dc.die()
