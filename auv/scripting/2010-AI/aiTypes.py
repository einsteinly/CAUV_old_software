

class Demand:
    def __init__(self):
        self.source = None
        self.priority = 0
    def execute(self, auv):
        # set autopilots etc
        pass
    def cleanup(self, auv):
        # implement this to clean-up anything that should be cleaned up 
        pass
