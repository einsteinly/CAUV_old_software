#!/usr/bin/env python2.7

import cauv
import cauv.node as node
import cauv.messaging as msg

import time

class ScriptResponseObs(msg.BufferedMessageObserver):
    def onScriptResponseMessage(self, m):
        print '>>>', m.response

n = node.Node("pyscript2")
n.join("gui") 
n.addObserver(ScriptResponseObs())

n.send(msg.ScriptMessage("hello, python", 10.0), "control")
n.send(msg.ScriptMessage("response('hello, python')", 10.0), "control")
n.send(msg.ScriptMessage("import cauv.messaging as msg", 10.0), "control")
n.send(msg.ScriptMessage("response(str(dir(msg)))", 10.0), "control")


time.sleep(5)

