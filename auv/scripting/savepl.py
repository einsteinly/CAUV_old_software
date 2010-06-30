#! /usr/bin/env python

#from __future__ import with_satement

import cauv.node as node
import cauv.messaging as messaging
import cauv.pipeline as pipeline

import time
import pickle
import optparse

def savepl(spread, fname):
    with open(fname, 'wb') as outf:
        print 'Connecting...'
        n = node.Node("py-plsave", spread)

        print 'Initializing pipeline model...'        
        model = pipeline.Model(n)
        #print 'Setting debug level to -3'
        #n.send(messaging.DebugLevelMessage(-3), "debug")

        print 'Getting pipeline state...'
        saved = model.get(3.0)
        
        print 'Pickling...'
        pickle.dump(saved, outf)

        print 'Done.'


def loadpl(spread, fname):
    with open(fname, 'rb') as inf:
        print 'Connecting...'        
        n = node.Node("py-plsave", spread)

        print 'Initializing pipeline model...'
        model = pipeline.Model(n)
        #print 'Setting debug level to -3'
        #n.send(messaging.DebugLevelMessage(-3), "debug")

        print 'UnPickling...'
        saved = pickle.load(inf)
        
        print 'Setting pipeline state...'
        model.set(saved)

        print 'Done.'


if __name__ == '__main__':
    print 'test:'
    e = messaging.NodeType(3)
    print e
    print 'test done.'

    parser = optparse.OptionParser()
    parser.add_option("-f", "--file", dest="fname", default="pipeline.pickle")
    parser.add_option("-s", "--spread", dest="spread", default="16707@localhost")
    (opts, args) = parser.parse_args()
    if len(args) == 0 or args[0].lower() == 'save':
        savepl(opts.spread, opts.fname)
    elif len(args) == 1:
        loadpl(opts.spread, opts.fname)
    else:
        print 'usage: savepl [-f filename] (load|save)'


