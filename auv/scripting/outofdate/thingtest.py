#!/usr/bin/env python2.7
#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

#pylint: disable=E0611

# to use this file you need to uncomment the Thing and ThingCaller exports in
# emit-static.cpp

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
