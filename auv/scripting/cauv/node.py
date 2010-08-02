#! /usr/bin/env python
#
# This module provides an as-user-friendly-as-possible wrapper around the
# 'messaging' module (which exports the c++ messaging interface directly)
#

import threading
from cauv import messaging

class Observer(messaging.BufferedMessageObserver):
    pass

class ServiceLevel:
    Unreliable = 0x01
    Reliable   = 0x02
    Fifo       = 0x04
    Causal     = 0x08
    Agreed     = 0x10
    Safe       = 0x20
    Regular    = 0X3f

#'''
#def addSynchronousPair(node, sendf, recvf, send_name = None, recv_name = None):
#    import new
#    if not send_name:
#        send_name = sendf.__name__
#    if not recv_name:
#        recv_name = recvf.__name__
#    send_method = new.instancemethod(sendf, node, node)
#    node.__dict__[send_name] = send_method
#    ...
#'''

class Node(messaging.CauvNode):
    def __init__(self, name, spreadserver="localhost", spreadport=16707):
        #print 'CauvNode.__init__ ...'
        messaging.CauvNode.__init__(self, name, spreadserver, spreadport)
        #print '__run ...'
        self.__run()

    def __run(self):
        t = threading.Thread(target=self.run)
        t.daemon = True
        t.start()
    
    def send(self, message, groups, service_level=ServiceLevel.Safe):
        self.mailbox.send(message, service_level, groups)

    #def send(self, message):
    #    self.mailbox.send(message, ServiceLevel.Safe, message.group())
    
    #def receive(self, timeout):
    #    return self.mailbox.receive(timeout)

