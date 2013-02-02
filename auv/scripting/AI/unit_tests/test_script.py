#!/usr/bin/env python2.7

import AI
from cauv.debug import debug, warning, error, info
import time

class NestedOptions(object):
    def __init__(self, a, b, c):
        self.a = a
        self.b = b
        self.c = c
    def to_dict(self):
        return {"a" : self.a,
                "b" : self.b,
                "c" : self.c}

    def from_dict(self, opts_dict):
        self.a = opts_dict["a"]
        self.b = opts_dict["b"]
        self.c = opts_dict["c"]

class Test(AI.Script):
    """Test Script Please Ignore"""
    class DefaultOptions(AI.Script.DefaultOptions):
        def __init__(self):
            self.succeed = False
            self.wait_seconds = 1
            self.debug_message = "Testing"
            self.nest = NestedOptions(1,2,3)

    def run(self):
        debug(self.options.debug_message)
        time.sleep(self.options.wait_seconds)
        debug("Nested A: {}".format(self.options.nest.a))
        debug("Nested B: {}".format(self.options.nest.b))
        debug("Nested C: {}".format(self.options.nest.c))
        return self.options.succeed

Script = Test

if __name__ == "__main__":
    Test.entry()
