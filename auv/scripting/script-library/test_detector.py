from AI_classes import aiDetector

class detector(aiDetector):
    def process(self):
        if raw_input('Has something been detected?'):
            self.detected = True
