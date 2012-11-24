# This module transparently wraps the boost-python exported messaging
# interface

from cauvinterface import *

#happens first so debug options can be parsed before any output occurs
import sys
debugParseOptions(sys.argv)

# until it works...
AIMessageObserver = BufferedMessageObserver

import copy_reg

#info('registering picklers...')

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
    warning('performing pickle test...')
    import pickle
    f = open('pickle-test.pickle', 'w')
    to_pickle = []
    to_pickle.append(MotorStateMessage(MotorID.Prop, 127))
    vo = NodeOutputArcVec()
    vo.append(NodeOutputArc(NodeInput(4, 'qwerty'), 'abcdef'))
    vi = NodeInputArcVec()
    vi.append(NodeInputArc('ABC\0DEF', NodeOutput(4, '123', OutputType.Image)))
    to_pickle.append(vi)
    to_pickle.append(vo)
    to_pickle.append(AddNodeMessage("test1", NodeType.FileInput, vi, vo))
    g = GraphDescriptionMessage()
    to_pickle.append(g)
    to_pickle.append(MotorStateMessage(MotorID.Prop, 127))
    pickle.dump(to_pickle, f)
    f.close()
    f = open('pickle-test.pickle')
    for thing in  pickle.load(f):
        print thing
    f.close()

#info('pickle registration complete')

