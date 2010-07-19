#! /usr/bin/env python

import sys
from cauv.messaging import Thing
from cauv.messaging import ThingCaller

caller = ThingCaller()
print "With nothing:", caller.callThing()

caller.setThing(Thing())
print "With Thing:", caller.callThing()

class MyThing(Thing):
    def foo(self):
        return 12365

caller.setThing(MyThing())
print "with MyThing:", caller.callThing()

t = Thing()

def foo2():
    return 19

t.foo = foo2
caller.setThing(t)
print "with Thing with foo=foo2:", caller.callThing()
