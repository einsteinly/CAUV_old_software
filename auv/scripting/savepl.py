#! /usr/bin/env python

#from __future__ import with_satement

import cauv.node as node
import cauv.messaging as messaging
import cauv.pipeline as pipeline
from cauv.debug import debug, warning, error, info

import time
import pickle
import optparse

def savepl(spread, port, fname, timeout=3.0):
    with open(fname, 'wb') as outf:
        info('Connecting...')
        n = node.Node("py-plsave", spread, port)

        info('Initializing pipeline model...')
        model = pipeline.Model(n)

        info('Getting pipeline state...')
        saved = model.get(timeout)
        
        info('Pickling...')
        pickle.dump(saved, outf)

        info('Done.')


def loadpl(spread, port, fname, timeout=3.0):
    with open(fname, 'rb') as inf:
        info('Connecting...')
        n = node.Node("py-plsave", spread, port)

        info('Initializing pipeline model...')
        model = pipeline.Model(n)

        info('UnPickling...')
        saved = pickle.load(inf)
        
        info('Setting pipeline state...')
        model.set(saved, timeout)

        info('Done.')

def clearpl(spread, port):
        info('Connecting...')
        n = node.Node("py-plsave", spread, port)

        info('Initializing pipeline model...')
        model = pipeline.Model(n)

        info('Clearing...')
        model.clear()

        info('Done.')


if __name__ == '__main__':
    parser = optparse.OptionParser(usage='usage: %prog [-f filename] (load|save|clear)')
    parser.add_option("-f", "--file", dest="fname", default="pipeline.pipe")
    parser.add_option("-s", "--spread", dest="spread", default="localhost")
    parser.add_option("-p", "--port", dest="port", type='int', default=16707)
    parser.add_option("-t", "--timeout", dest="timeout", type='float', default=3.0)

    (opts, args) = parser.parse_args()
    port = opts.port
    fname = opts.fname
    timeout = opts.timeout

    if len(args) == 0 or args[0].lower() == 'save':
        savepl(opts.spread, port, fname, timeout)
    elif len(args) == 1 and args[0].lower() == 'load':
        loadpl(opts.spread, port, fname, timeout)
    elif len(args) == 1 and args[0].lower() == 'clear':
        clearpl(opts.spread, port)
    else:
         parser.print_help()


