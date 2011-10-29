#!/usr/bin/env python2.7

#from __future__ import with_satement

import cauv.node as node
import cauv.messaging as messaging
import cauv.pipeline as pipeline
from cauv.debug import debug, warning, error, info

import time
import pickle
import argparse

def savepl(args, fname, timeout=3.0, name='default'):
    with open(fname, 'wb') as outf:
        info('Connecting...')
        n = node.Node("py-plsave", args)
        try:
            info('Initializing pipeline model (%s)...' % name)
            model = pipeline.Model(n, name)

            info('Getting pipeline state...')
            saved = model.get(timeout)
            
            info('Pickling...')
            pickle.dump(saved, outf)
        
            info('Done.')
        finally:
            n.stop()


def loadpl(args, fname, timeout=3.0, name='default'):
    with open(fname, 'rb') as inf:
        info('Connecting...')
        n = node.Node("py-plsave", args)
        try:
            info('Initializing pipeline model (%s)...' % name)
            model = pipeline.Model(n, name)

            info('UnPickling...')
            saved = pickle.load(inf)
            
            info('Setting pipeline state...')
            model.set(saved, timeout)

            info('Done.')
        finally:
            n.stop()

def clearpl(args, name='default'):
        info('Connecting...')
        n = node.Node("py-plsave", args)
        try:
            info('Initializing pipeline model (%s)...' % name)
            model = pipeline.Model(n, name)

            info('Clearing...')
            model.clear()

            info('Done.')
        finally:
            n.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(usage='usage: %prog [-f filename] (load|save|clear)')
    parser.add_argument("-f", "--file", dest="fname", default="pipeline.pipe")
    parser.add_argument("-n", "--pipeline-name", dest="name", default="default")
    parser.add_argument("-t", "--timeout", dest="timeout", type='float', default=3.0)

    opts, args = parser.parse_known_args()
    fname = opts.fname
    timeout = opts.timeout
    name = opts.name

    if len(args) == 1 and args[0].lower() == 'save':
        savepl(args, fname, timeout, name)
    elif len(args) == 1 and args[0].lower() == 'load':
        loadpl(args, fname, timeout, name)
    elif len(args) == 1 and args[0].lower() == 'clear':
        clearpl(args, port, name)
    else:
         parser.print_help()


