#! /usr/bin/env python

import sys
sys.path.append("/home/jc593/Dev/hg-code/auv/scripting/bin/gcc-4.2.4/debug/")

import cauv
import threading

n = cauv.CauvNode("pycauv", "16707@localhost")
print "created cauv node"

t = threading.Thread(target=n.run)
t.daemon = True
print "created thread object"

t.start()
print "started cauv node run thread"

n.mailbox.join("test")
print "joined test group"

n.mailbox.join("pipeline")
print "joined pipeline group"

n.mailbox.join("invalid group")
print "attempted to join an invalid group"

mo = cauv.MessageObserver()
print mo, "created MessageObserver"

dmo = cauv.DebugMessageObserver()
print dmo, "created DebugMessageObserver"

class MMO(cauv.MessageObserver):
    def onDebugMessage(self, msg):
        print "mesage received:", msg
m = MMO()
print m, "created MessageObserver with overload"

n.monitor.addObserver(m)
print "added observer with overload"

n.monitor.addObserver(mo)
print "added empty observer"

n.monitor.addObserver(dmo)
print "added debug observer"

msg = cauv.DebugMessage(cauv.DebugType.Debug, "test message string!")
print "created debug message:", msg

for i in xrange(1, 101):
    try:
        print "Sending message", i
        n.mailbox.send(msg, 1, "test")
        #print n.mailbox.receive(1000)
    except Exception, e:
        print e

del dmo
del mo
del m
del msg
del n

