#!/usr/bin/env python2.7

import cauv
import cauv.messaging as msg
import cauv.node as node
import cauv.control
import cauv.pipeline

from cauv.debug import debug, warning, error, info

import subprocess
import threading
import copy
import traceback
import time
import Queue

#class Script:
#    def __init__(self, script, timeout, id, seq):
#        self.script = script
#        self.timeout = timeout
#        self.id = id
#        self.seq = seq
#
#    def __repr__(self):
#        return "%s %s" % (self.id, self.seq)

class ScriptObserver(msg.MessageObserver, threading.Thread):
    def __init__(self, node):
        msg.MessageObserver.__init__(self)
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
            "msg" : cauv.messaging,
            "run" : self.runScript,
            "help" : self.printHelp
        }

    def send(self, m):
        self.__node.send(m, "gui")
    
    def onScriptMessage(self, m):
        script = m.request;
        debug('received script: %s' % script)
        self.eval_queue.put(script)

    def sendScriptResponse(self, script, level, m):
        debug('sending script response to %s: %s' % (script, str(m)))
        response = msg.ScriptResponse();
        response.response = str(m)
        response.level = level
        response.id = script.id
        response.seq = script.seq
        self.send(msg.ScriptResponseMessage(response))

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
    
    def runScript(self, sendFunc, script_name):
        #subprocess.popen('/bin/sh ./run.sh ./script_library/%s' % script_name)
        sendFunc('not implemented')
    
    def printHelp(self, sendFunc):
        sendFunc('''
Available objects and functions:
    response(message, level=msg.DebugType.Info) # sends response to GUI
    node     # cauv.control.Node object
    auv      # cauv.control.AUV
    control  # cauv.control module
    pipeline # cauv.cauv.pipeline.Model object: use to load and save pipelines
    msg      # cauv.messaging module: C++ exposed types are msg.TypeName
    run(script name) # NOT COMPLETE: run a script from the library
    help()   # print this help message
''')
        

def main():
    import sys
    n = node.Node("pyscript",sys.argv[1:])
    try:
        so = ScriptObserver(n)
        while True:
            time.sleep(0.2)
    finally:
        n.stop()

if __name__ == '__main__':
    main()

