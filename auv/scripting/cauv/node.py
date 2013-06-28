#!/usr/bin/env python2.7
#
# This module provides an as-user-friendly-as-possible wrapper around the
# 'messaging' module (which exports the c++ messaging interface directly)
#
from cauv import messaging
from cauv.debug import debug, error, warning, info, setDebugName
from fileCached import fileCached
import threading
import traceback
import sys
import os
import atexit

class Observer(messaging.MessageObserver):
    pass

@fileCached(30)
def getVersionInfo():
    import os, shlex, subprocess
    repo_root = '/'.join(
        (os.path.join(os.getcwd(), __file__)).split('/')[:-4]
    )
    hg_cmdstr = 'hg -R %s %%s' % repo_root
    diff_cmdstr = hg_cmdstr % ('diff %s/auv/scripting/' % repo_root)
    #summ_cmdstr = hg_cmdstr % 'summary'
    dp = subprocess.Popen(shlex.split(diff_cmdstr), stdout = subprocess.PIPE)
    #sp = subprocess.Popen(shlex.split(summ_cmdstr), stdout = subprocess.PIPE)
    diff = dp.communicate()[0]
    #summary = sp.communicate()[0]
    summary = ''
    return (summary, diff)

class Node(messaging.CauvNode):
    def __init__(self, name, args = None, run_now = True):
        setDebugName(name)
        info('CAUV Python Node Initialisation...') 
        messaging.CauvNode.__init__(self, name)
        self.__t = None
        try:
            if os.getenv('CAUV_SHUTUP') is not None:
                lc = getVersionInfo()[1]
                if lc:
                    warning('Running with uncommitted local changes:\n%s' % lc)
        except IOError:
            # stupid OS X... apparently my os module was compiled wrong
            pass
        if args is None:
            #this sets up default options
            self.parseOptions(sys.argv[0:1])
        else:
            self.parseOptions(sys.argv[0:1] + args)

        if run_now:
            self.run(False)
    
    def stop(self):
        debug('stopping messaging thread...')
        messaging.CauvNode.stop(self)
        debug('joining messaging thread...')
        try:
            if self.__t is not None:
                self.__t.join()
        except RuntimeError:
            #tried to join current thread
            pass

    def __run(self):
        debug('cauv.Node running')   
        atexit.register(self.stop)
        try:
            messaging.CauvNode.run(self,True)
        except:
            error(traceback.format_exc())
            self.stop()
        finally:
            debug('CAUV Node stopped')

    def run(self, synchronous = True):
        if synchronous:
            self.__run()
        else:
            self.__t = threading.Thread(target=self.__run)
            self.__t.daemon = True
            self.__t.start()

    def send(self, message, groups=None, service_level=messaging.MessageReliability.RELIABLE_MSG):
        if groups == None:
            groups = message.group
        self.mailbox.send(message, service_level, groups) #pylint: disable=E1101
