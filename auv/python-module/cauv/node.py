#! /usr/bin/env python
#
# This module provides an as-user-friendly-as-possible wrapper around the
# 'messaging' module (which exports the c++ messaging interface directly)
#
import threading
import traceback
from cauv import messaging
from cauv.debug import debug, error, warning, info

#pylint: disable=E1101

class Observer(messaging.MessageObserver):
    pass

class ServiceLevel:
    Unreliable = 0x01
    Reliable   = 0x02
    Fifo       = 0x04
    Causal     = 0x08
    Agreed     = 0x10
    Safe       = 0x20
    Regular    = 0X3f

def getVersionInfo():
    import os
    #repo_root = '/'.join(
    #    (os.path.join(os.getcwd(), __file__)).split('/')[:-5]
    #)
    diff = ''
    summary = ''
    #(si, so) = os.popen2('hg -R %s summary --color=yes' % repo_root)
    #summary = so.read()
    #(si, so) = os.popen2('hg -R %s diff --color=yes' % repo_root)
    #diff = so.read()
    return (summary, diff)

class Node(messaging.CauvNode):
    def __init__(self, name, spreadserver="localhost", spreadport=16707):
        info('CAUV Python Node Initialisation...') 
        try:
            lc = getVersionInfo()[1]
            if lc:
                warning('Running with uncommitted local changes:\n%s' % lc)
        except IOError:
            # stupid OS X... apparently my os module was compiled wrong
            pass
        messaging.CauvNode.__init__(self, name, spreadserver, spreadport)
        self.__run()

    def __del__(self):
        debug('CAUV Node __del__...')
        debug('CAUV Node Stopping...')
        self.stop()
        if self.__t.isAlive():
            debug('CAUV Node Joining run thread...')
            self.__t.join()
        debug('CAUV Node __del__ complete')
    
    def __callRunWithTryFinally(self):
        try:
            self.run()
        except Exception, e:
            error(traceback.format_exc())
        finally:
            self.stop()
            debug('CAUV Node clearing up...')
        debug('CAUV Node run thread exiting...')

    def __run(self):
        debug('CauvNode.__run...')   
        self.__t = threading.Thread(target=self.__callRunWithTryFinally)
        # TODO: False?
        self.__t.daemon = True
        self.__t.start()
    
    def send(self, message, groups=None, service_level=ServiceLevel.Safe):
        if groups == None:
            groups = message.group
        self.mailbox.send(message, service_level, groups)
    
    #def receive(self, timeout):
    #    return self.mailbox.receive(timeout)

