#! /usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node as node
import cauv.control
import cauv.pipeline

from cauv.debug import debug, warning, error, info

import threading
import copy
import traceback
import time
import Queue

class Script:
    def __init__(self, script, timeout, id, seq):
        self.script = script
        self.timeout = timeout
        self.id = id
        self.seq = seq

    def __repr__(self):
        return "%s %s" % (self.id, self.seq)

class ScriptObserver(msg.BufferedMessageObserver, threading.Thread):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        threading.Thread.__init__(self)
        self.__node = node
        node.join("control")
        node.addObserver(self)
        self.eval_queue = Queue.Queue(1000)
        self.daemon = True
        self.start()

    def resetEnvironment(self, script):
        #node = cauv.node.Node("pymote")
        auv = cauv.control.AUV(self.__node)
        plmodel = cauv.pipeline.Model(self.__node)
        def sendFunc(msg, level=msg.DebugType.Info):
            print 'send:', msg, level
            self.sendScriptResponse(script, level, msg)
        self.eval_context_locals = { }
        self.eval_context_globals = {
            "response": sendFunc,
            "node" : self.__node,
            "auv" : auv,
            "control" : cauv.control,
            "pipeline" : plmodel,
            "msg" : cauv.messaging
        }

    def send(self, m):
        self.__node.send(m, "gui")
    
    def onScriptMessage(self, m):
        script = Script(m.script, m.timeout, m.id, m.seq)
        debug('received script: %s' % script)
        self.eval_queue.put(script)

    def sendScriptResponse(self, script, level, m):
        debug('sending script response to %s: %s' % (script, str(m)))
        self.send(msg.ScriptResponseMessage(str(m), level, script.id, script.seq))

    def run(self):
        info('script queue thread started')
        while True:
            script = self.eval_queue.get()
            self.evaluate(script)

    def evaluate(self, script):
        # TODO: apply timeout... somehow
        warning('timeout is ignored')
        debug('evaluating script %s:\n%s\n#[end script]' % (script, script.script))
        self.resetEnvironment(script)
        with open('remote.py.script.tmp', 'w+') as tmpf:
            tmpf.write(script.script)
        try:
            r = execfile('remote.py.script.tmp',
                         self.eval_context_globals,
                         self.eval_context_globals)
        except Exception:
            message = 'error in script:\n' + traceback.format_exc() 
            error(message)
            self.sendScriptResponse(script, msg.DebugType.Error, message)
        else:
            message = 'script returned: ' + str(r)
            info(message)
            self.sendScriptResponse(script, msg.DebugType.Info, message)

def main():
    n = node.Node("pyscript")
    so = ScriptObserver(n)
    while True:
        time.sleep(1.0)

if __name__ == '__main__':
    main()

