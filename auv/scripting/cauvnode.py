#! /usr/bin/env python
#
# This module provides an as-user-friendly-as-possible wrapper around the
# 'cauv' module (which exports the c++ messaging interface directly)
#

import sys
sys.path.append("/home/jc593/Dev/hg-code/auv/scripting/bin/gcc-4.2.4/debug/")

import cauv
import threading

class Observer(cauv.BufferedMessageObserver):
    pass

class ServiceLevel:
    Unreliable = 0x01
    Reliable   = 0x02
    Fifo       = 0x04
    Causal     = 0x08
    Agreed     = 0x10
    Safe       = 0x20
    Regular    = 0X3f

class Node(cauv.CauvNode):
    def __init__(self, name):
        cauv.CauvNode.__init__(self, name, "16707@localhost")
        self.__run()

    def __run(self):
        t = threading.Thread(target=self.run)
        t.daemon = True
        t.start()

    def join(self, group):
        self.mailbox.join(group)
    
    def send(self, message, groups, service_level=ServiceLevel.Safe):
        self.mailbox.send(message, service_level, groups)
    
    def receive(self, timeout):
        return self.mailbox.receive(timeout)

    def addObserver(self, observer):
        self.monitor.addObserver(observer)

