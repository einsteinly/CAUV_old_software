#! /usr/bin/env python
import cauv.node as node
import cauv.messaging as messaging
import cauv.pipeline as pipeline

import time

def main():
    n = node.Node("pycauv")
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
    m.setDoubleBuffered(messaging.MessageType.Status, True)
    m.setDoubleBuffered(messaging.MessageType.Image, True)
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

    time.sleep(3.0)
    
    n.mailbox.join("pipeline")
    n.mailbox.join("pl_gui")
    n.mailbox.join("images")
    
    n.send(messaging.ClearPipelineMessage(), "pipeline")
    #print n.receive(1000)
    n.send(messaging.AddNodeMessage(messaging.NodeType.FileInput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")
    #print n.receive(1000)
    
    n.send(messaging.AddNodeMessage(messaging.NodeType.GuiOutput,
                                    messaging.NodeInputArcVec(),
                                    messaging.NodeOutputArcVec()), "pipeline")
    
    model = pipeline.Model(n)
    
    print 'Getting pipeline state...'
    saved = model.get()
    print 'before:', saved
    print 'Clearing pipeline state...'
    model.clear()
    print 'Setting pipeline state...'
    model.set(saved)
    print 'after:', model.get()

    
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
    main()


