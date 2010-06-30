#! /usr/bin/env python

import cauv
import cauv.messaging as msg
import cauv.node as node

import threading
import copy
import traceback
import time

class ScriptObserver(msg.BufferedMessageObserver, threading.Thread):
    def __init__(self, node):
        msg.BufferedMessageObserver.__init__(self)
        threading.Thread.__init__(self)
        self.__node = node
        node.addObserver(self)
        self.eval_context_locals = {}
        self.eval_context_globals = {"msg": self.sendScriptResponse}
        self.eval_queue = []
        self.eval_queue_condition = threading.Condition()
        # TODO: check this line:
        self.daemon = True
        self.start()

    def send(self, m):
        self.__node.send(m, "gui")
    
    def onScriptMessage(self, m):
        print 'received script:\n%s\n#[end script]' % m.script
        self.eval_queue.append(m.script, m.timeout)

    def sendScriptResponse(self, msg):
        self.send(msg.ScriptResponseMessage(msg))

    def run(self):
        print 'script queue thread started'
        while True:
            script = None
            self.eval_queue_condition.acquire()
            self.eval_queue_condition.wait()
            script = self.eval_queue.pop()
            self.eval_queue_condition.release()
            evaluate(script)

    def evaluate(self, script):
        print 'evaluating script\n%s\n#[end script]' % m.script
        try:
            r = eval(script, self.eval_context_globals, self.eval_context_globals)
        except Exception:
            print 'error:', traceback.format_exc()
        else:
            print 'script returned:', r

def main():
    n = node.Node("pyscript")
    so = ScriptObserver(n)
    while True:
        time.sleep(1000)

if __name__ == '__main__':
    main()

