from AI_classes import aiDetector

class detector(aiDetector):
    def process(self):
        #This is an example of what NOT to put in a process command, as this holds up the entire detector_process, preventing everything else from happening.
        if raw_input('Has something been detected?'):
            self.detected = True
