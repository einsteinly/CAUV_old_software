#!/usr/bin/env python2.7

import AI
import time
import cauv.messaging as msg
from cauv.debug import debug, info, warning, error

class TestDetector(AI.Detector):
    class DefaultOptions(AI.Detector.DefaultOptions):
        def __init__(self):
            self.wait_seconds = 1
            self.fire_timeout = 10
            self.fire = False

    def run(self):
        while True:
            time.sleep(self.options.wait_seconds)
            if self.options.fire:
                self.fire(self.options.fire_timeout)

Detector = TestDetector

if __name__ == "__main__":
    TestDetector.entry()
