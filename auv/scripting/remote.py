#! /usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node as node
import cauv.control
import cauv.pipeline

import threading
import copy
import traceback
import time
import Queue

class ScriptObserver(msg.BufferedMessageObserver, threading.Thread):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        threading.Thread.__init__(self)
        self.__node = node
        node.join("control")
        node.addObserver(self)
        self.resetEnvironment()
        self.eval_queue = Queue.Queue(1000)
        # TODO: check this line:
        self.daemon = True
        self.start()

    def resetEnvironment(self):
        #node = cauv.node.Node("pymote")
        auv = cauv.control.AUV(self.__node)
        plmodel = cauv.pipeline.Model(self.__node)
        self.eval_context_locals = { }
        self.eval_context_globals = {
            "response": self.sendScriptResponse,
            "node" : self.__node,
            "auv" : auv,
            "control" : cauv.control,
            "pipeline" : plmodel,
            "msg" : cauv.messaging
        }

    def send(self, m):
        self.__node.send(m, "gui")
    
    def onScriptMessage(self, m):
        print 'received script:\n%s\n#[end script]' % m.script
        self.eval_queue.put((m.script, m.timeout))

    def sendScriptResponse(self, m):
        print 'sending script response:', str(m)
        self.send(msg.ScriptResponseMessage(str(m)))

    def run(self):
        print 'script queue thread started'
        while True:
            (script, timeout) = self.eval_queue.get()
            self.evaluate(script, timeout)

    def evaluate(self, script, timout):
        # TODO: apply timeout... somehow
        print 'evaluating script\n%s\n#[end script]' % script
        with open('remote.py.script.tmp', 'w+') as tmpf:
            tmpf.write(script)
        try:
            r = execfile('remote.py.script.tmp',
                         self.eval_context_globals,
                         self.eval_context_globals)
        except Exception:
            print 'error:', traceback.format_exc()
            self.sendScriptResponse('error:' + traceback.format_exc())
        else:
            print 'script returned:', str(r)
            self.sendScriptResponse('script returned:' + str(r))            

def main():
    n = node.Node("pyscript")
    so = ScriptObserver(n)
    while True:
        time.sleep(1000)

if __name__ == '__main__':
    main()

