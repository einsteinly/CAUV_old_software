#!/usr/bin/env python

import cauv
import cauv.node as node
import cauv.messaging as msg


class ScriptResponseObs(msg.BufferedMessageObserver):
    def onScriptResponseMessage(self, m):
        print '>>>', m.response

n = node.Node("pyscript2")
n.addObserver(ScriptResponseObs())
n.send(msg.ScriptMessage("hello, python", 10.0), "control")


