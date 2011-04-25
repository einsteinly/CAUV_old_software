import cauv
import cauv.messaging as messaging
import cauv.node
from cauv.debug import debug, warning, error, info

import AI_messages

class detectionControl():
    def __init__(self):
        self.node = cauv.node.Node('pydetcon')
        self.ai_node = AI_messages.ai_node(self.node, process_initial='d')
        self.message_map = {
                        'start':'start',
                        'stop':'stop',
                        }
        self.modules = []
        self.running_detectors = {} 
    def start(self, detection_file):
        if detection_file in self.running_detectors:
            debug("Detection class "+detection_file+" is already running.")
            return
        elif not (detection_file in self.modules):
            self.modules[detection_file] = __import__(detection_file)
        self.running_detectors[detection_file] = self.modules[detection_file].detector()
    def stop(self, detection_file):
        try:
            self.running_detectors[detection_file].die()
            self.running_detectors.pop(detection_file)
        except KeyError:
            debug(detection_file+" is not runnning, so cannot be stopped")
    def run(self):
        while True:
            #check for messages
            while True:
                message = self.ai_node.getMessage(block=False)
                if message:
                    try:
                        result = self.__getattr__(self.message_map[message[2]])(*message[3])
                    except:
                        debug('Could not interpret message to detection process')
                        result = None
                else:
                    break
            #send status
            self.ai_node.send('t','update_detectors',self.running_detectors.keys())
            #run detection
            for detector in self.running_detectors:
                detector.process()

if __name__ == '__main__':
    dc = detectionControl()
    dc.run()
