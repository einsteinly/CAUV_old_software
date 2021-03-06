#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#


import cauv
import cauv.node as node
import cauv.messaging as msg

import time
import sys

class ScriptResponseObs(msg.BufferedMessageObserver):
    def onScriptResponseMessage(self, m):
        print '>>>', m.response

n = node.Node("pyscript2",sys.argv[1:])
n.join("gui") 
n.addObserver(ScriptResponseObs())

n.send(msg.ScriptMessage("hello, python", 10.0), "control")
n.send(msg.ScriptMessage("response('hello, python')", 10.0), "control")
n.send(msg.ScriptMessage("import cauv.messaging as msg", 10.0), "control")
n.send(msg.ScriptMessage("response(str(dir(msg)))", 10.0), "control")


time.sleep(5)

