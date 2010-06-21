#! /usr/bin/env python
import cauvnode
import cauv

import time

def main():
    n = cauvnode.Node("pycauv")
    print "created cauvnode.Node"

    n.join("test")
    print "joined test group"

    n.join("pipeline")
    print "joined pipeline group"

    n.join("invalid group")
    print "attempted to join an invalid group"

    o = cauvnode.Observer()
    print o, "created cauvnode.Observer"

    dmo = cauv.DebugMessageObserver()
    print dmo, "created DebugMessageObserver"

    class MMO(cauvnode.Observer):
        def onDebugMessage(self, msg):
            print "mesage received:", msg
        def onImageMessageBuffered(self, msg):
            print "mesage received (buffered):", msg
            print "sleeping..."
            time.sleep(500)
        def onNodeAddedMessage(self, msg):
            print "mesage received:", msg
        def onStatusMessageBuffered(self, msg):
            print "message received (buffered):", msg

    m = MMO()
    m.setDoubleBuffered(cauv.MessageType.Status, True)
    m.setDoubleBuffered(cauv.MessageType.Image, True)
    print m, "created Observer with overload"

    n.addObserver(m)
    print "added observer with overload" 
    n.addObserver(o)
    print "added empty observer" 
    n.addObserver(dmo)
    print "added debug observer"

    msg = cauv.DebugMessage(cauv.DebugType.Debug, "test message string!")
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
    
    n.mailbox.join("pipeline")
    n.mailbox.join("pl_gui")
    n.mailbox.join("images")
    
    n.send(cauv.ClearPipelineMessage(), "pipeline")
    print n.receive(1000)
    n.send(cauv.AddNodeMessage(cauv.NodeType.FileInput,
                               cauv.NodeInputArcVec(),
                               cauv.NodeOutputArcVec()), "pipeline")
    print n.receive(1000)

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


