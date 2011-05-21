from cauv.debug import debug, warning, error, info

import threading
import time

from AI_classes import aiProcess, external_function

class detectionControl(aiProcess):
    def __init__(self):
        self.external_functions = ['start', 'stop']
        aiProcess.__init__(self, 'detector_control')
        self.modules = {}
        self.running_detectors = {}
        self.request_lock = threading.Lock()
        self.start_requests = []
        self.stop_requests = []
        self.enable_flag = threading.Event()
        self.enable_flag.set()
    @external_function
    def start(self, detection_file):
        with self.request_lock:
            self.start_requests.append(detection_file)
    @external_function
    def stop(self, detection_file):
        with self.request_lock:
            self.stop_requests.append(detection_file)
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
                for running_detector in self.running_detectors:
                    running_detector.die()
                self.running_detectors = {}
                info('Detector process disabled')
                self.enable_flag.wait()
            #update running detectors from requests (has to be done here as list of running detectors is constantly in use by this process)
            with self.request_lock:
                for detection_file in self.start_requests:
                    if detection_file in self.running_detectors:
                        debug("Detection class %s is already running." %(detection_file))
                    else:
                        if not (detection_file in self.modules):
                            #interesting behaviour of __import__ here
                            self.modules[detection_file] = __import__('detector_library.'+detection_file, fromlist=['detector_library'])
                        self.running_detectors[detection_file] = self.modules[detection_file].detector(self.node)
                        info("Started detection class %s." %(detection_file))
                for detection_file in self.stop_requests:
                    try:
                        self.running_detectors[detection_file].die()
                        self.running_detectors.pop(detection_file)
                        info("Stopped detection class %s." %(detection_file))
                    except KeyError:
                        debug(detection_file+" is not runnning, so cannot be stopped")
            #need to sleep  or else it goes crazy when not much processing to do
            time.sleep(1)
            #send status
            self.ai.task_manager.update_detectors(self.running_detectors.keys())
            #run detection
            for detection_file in self.running_detectors:
                #since each processing could take a while, and disabling needs to be pretty fast, check here
                if not self.enable_flag.is_set():
                    break
                self.running_detectors[detection_file].process()
                if self.running_detectors[detection_file].detected:
                    self.ai.task_manager.notify_detector(detection_file, True)

if __name__ == '__main__':
    dc = detectionControl()
    dc.run()
