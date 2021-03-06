#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import cauv.node as node
import cauv.messaging as messaging
import cauv.pipeline as pipeline

import time

def pipelineTest():
    import sys
    n = node.Node("pycauv-pl",sys.argv[1:])
    model = pipeline.Model(n)
    print 'Setting debug level to -3'
    n.send(messaging.DebugLevelMessage(-3), "debug")
    
    print 'Adding stuff to the pipeline...'
    n.send(messaging.ClearPipelineMessage(), "pipeline")
    n.send(messaging.AddNodeMessage(messaging.NodeType.FileInput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")
    n.send(messaging.AddNodeMessage(messaging.NodeType.GuiOutput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")

    print 'Getting pipeline state...'
    saved = model.get()
    print 'before:', saved
    print 'Clearing pipeline state...'
    model.clear()
    print 'Setting pipeline state...'
    model.set(saved)
    print 'after:', model.get()

def randomTest():
    import sys
    n = node.Node("pycauv",sys.argv[1:])
    print "created node.Node"

    n.join("test")
    print "joined test group"

    n.join("pipeline")
    print "joined pipeline group"

    n.join("invalid group")
    print "attempted to join an invalid group"

    o = node.Observer()
    print o, "created node.Observer"

    dmo = messaging.DebugMessageObserver()
    print dmo, "created DebugMessageObserver"

    class MMO(node.Observer):
        def onDebugMessage(self, msg):
            print "mesage received:", msg
        def onImageMessageBuffered(self, msg):
            print "image received:", msg
        def onNodeAddedMessage(self, msg):
            print "node added:", msg.nodeId, msg.nodeType
        def onStatusMessageBuffered(self, msg):
            print "node status:", msg.nodeId, msg.status

    m = MMO()
    m.setDoubleBuffered(messaging.MessageType.Status, True) #pylint: disable=E1101
    m.setDoubleBuffered(messaging.MessageType.Image, True) #pylint: disable=E1101
    print m, "created Observer with overload"

    n.addObserver(m)
    print "added observer with overload" 
    n.addObserver(o)
    print "added empty observer" 
    n.addObserver(dmo)
    print "added debug observer"

    msg = messaging.DebugMessage(messaging.DebugType.Debug, "test message string!")
    print "created debug message:", msg

    for i in xrange(1, 11):
        try:
            print "setting message string"
            msg.msg = "msg string %d" % i
            print "Sending message", i, msg
            n.send(msg, "test")
            #print n.mailbox.receive(1000)
        except Exception, e:
            print e

    time.sleep(3)
    n.send(messaging.ClearPipelineMessage("default"), "pipeline")
    n.send(messaging.AddNodeMessage("default",
                                    messaging.NodeType.FileInput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")
    n.send(messaging.AddNodeMessage("default",
                                    messaging.NodeType.GuiOutput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")

    print "deleting dmo", dmo
    del dmo
    print "deleting o", o
    del o
    print "deleting m", m
    del m
    print "deleting msg", msg
    del msg
    print "deleting n", n
    del n
    print "done."


if __name__ == '__main__':
    pipelineTest()


