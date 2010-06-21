#! /usr/bin/env python

import sys
sys.path.append("/home/jc593/Dev/hg-code/auv/scripting/bin/gcc-4.2.4/debug/")
import cauv

caller = cauv.ThingCaller()
print "With nothing:", caller.callThing()

caller.setThing(cauv.Thing())
print "With Thing:", caller.callThing()

class MyThing(cauv.Thing):
    def foo(self):
        return 12365

caller.setThing(MyThing())
print "with MyThing:", caller.callThing()

t = cauv.Thing()

def foo2():
    return 19

t.foo = foo2
caller.setThing(t)
print "with Thing with foo=foo2:", caller.callThing()
