from cauv.debug import debug, warning, error, info

import threading
import time
import traceback

from AI_classes import aiProcess, external_function, aiDetectorOptions

class detectionControl(aiProcess):
    def __init__(self):
        self.external_functions = ['start', 'stop']
        aiProcess.__init__(self, 'detector_control')
        self.modules = {}
        self.running_detectors = {}
        self.request_lock = threading.Lock()
        self.pl_requests = []
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
    @external_function
    def update_pl_requests(self, requests):
        with self.request_lock:
            self.pl_requests = requests
    def run(self):
        while True:
            if not self.enable_flag.is_set():
                for running_detector in self.running_detectors.values():
                    running_detector.die()
                self.running_detectors = {}
                self.ai.pipeline_manager.set_detector_pl([])
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
                            try:
                                self.modules[detection_file] = __import__('detector_library.'+detection_file, fromlist=['detector_library'])
                            except Exception:
                                error('Could not import detector %s.' %(detection_file,))
                                traceback.print_exc()
                                continue
                        try:
                            opts = self.modules[detection_file].detectorOptions({})
                        except Exception:
                            error('Could not initialise detector %s options.' %(detection_file,))
                            traceback.print_exc()
                            opts = aiDetectorOptions({})
                        try:
                            self.running_detectors[detection_file] = self.modules[detection_file].detector(self.node, opts)
                        except Exception:
                            error('Could not initialise detector %s.' %(detection_file,))
                            traceback.print_exc()
                            continue
                        info("Started detection class %s." %(detection_file))
                for detection_file in self.stop_requests:
                    try:
                        try:
                            self.running_detectors[detection_file].die()
                        except Exception as e:
                            if isinstance(e, KeyError):
                                raise e
                            error('Could not kill detector %s.' %(detection_file,))
                            traceback.print_exc()
                        self.running_detectors.pop(detection_file)
                        info("Stopped detection class %s." %(detection_file))
                    except KeyError:
                        debug(detection_file+" is not runnning, so cannot be stopped")
                #compile list of pls should be running
                pls = []
                for detector in self.running_detectors.values():
                    for pl_name, req in detector._pl_requests.items():
                        if req:
                            pls.append(pl_name)
                #sort out differences
                if set(pls) != set(self.pl_requests):
                    self.ai.pipeline_manager.set_detector_pl(pls)
                self.start_requests = []
                self.stop_requests = []
            #need to sleep  or else it goes crazy when not much processing to do
            time.sleep(1)
            #send status
            self.ai.task_manager.update_detectors(self.running_detectors.keys())
            #run detection
            for detection_file in self.running_detectors:
                #since each processing could take a while, and disabling needs to be pretty fast, check here
                if not self.enable_flag.is_set():
                    break
                try:
                    self.running_detectors[detection_file].process()
                except Exception:
                    error('Exception while running %s.' %(detection_file,))
                    traceback.print_exc()
                if self.running_detectors[detection_file].detected:
                    self.ai.task_manager.notify_detector(detection_file, True)

if __name__ == '__main__':
    dc = detectionControl()
    dc.run()
