# This module transparently wraps the boost-python exported messaging
# interface

from cauvinterface import *

#pylint: disable=E0602

# until it works...
AIMessageObserver = BufferedMessageObserver

import copy_reg

# enum picklers:
# http://stackoverflow.com/questions/3214969/pickling-an-enum-exposed-by-boost-python
# Modifications are to support the fact we load cauvinterface into the global
# namespace
def _isEnumType(o):
    return isinstance(o, type) and issubclass(o,int) and not (o is int)

def _tuple2enum(enum, value):
    enum = globals()[enum]
    e = enum.values.get(value,None)
    if e is None:
        e = enum(value)
    return e

def _registerEnumPicklers():
    from copy_reg import constructor, pickle
    def reduce_enum(e):
        enum = type(e).__name__.split('.')[-1]
        return ( _tuple2enum, ( enum, int(e) ) )
    constructor( _tuple2enum)
    for e in [ e for e in globals().itervalues() if _isEnumType(e) ]:
        #print 'registering pickler for %s', e
        pickle(e, reduce_enum)

_registerEnumPicklers()



# test
if False:
    import pickle
    f = open('enum-test.pickle', 'w')
    pickle.dump(MotorStateMessage(MotorID.Prop, 127), f)
    f.close()
    f = open('enum-test.pickle')
    print pickle.load(f)
    f.close()



