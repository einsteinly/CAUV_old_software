#!/usr/bin/env python2.7

#from __future__ import with_satement

import cauv.node
import cauv.messaging as messaging
import cauv.pipeline as pipeline
from cauv.debug import debug, warning, error, info

import time
import argparse

def savepl(node, fname, timeout=3.0, name='default'):
    with open(fname, 'wb') as outf:
        info('Initializing pipeline model (%s)...' % name)
        model = pipeline.Model(node, name)

        info('Getting pipeline state...')
        saved = model.get(timeout)
        
        info('Pickling...')
        model.dumpFile(saved, outf)


def loadpl(node, fname, timeout=3.0, name='default', clear=True):
    with open(fname, 'rb') as inf:
        info('Initializing pipeline model (%s)...' % name)
        model = pipeline.Model(node, name)

        info('UnPickling...')
        saved = model.loadFile(inf)
        
        info('Setting pipeline state...')
        model.set(saved, timeout, clear)

def clearpl(node, name='default'):
    info('Initializing pipeline model (%s)...' % name)
    model = pipeline.Model(node, name)

    info('Clearing...')
    model.clear()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", "--file", dest="fname", default="pipeline.pipe")
    parser.add_argument("-n", "--pipeline-name", dest="name", default="default")
    parser.add_argument("-t", "--timeout", dest="timeout", type=float, default=3.0)
    parser.add_argument('clear', action='store_true', default=False)
    parser.add_argument('verb', choices=('load', 'save', 'clear', 'add'))

    opts, unknown_args = parser.parse_known_args()
    fname = opts.fname
    timeout = opts.timeout
    name = opts.name
    
    info('Connecting...')
    node = cauv.node.Node("py-plsave", unknown_args)
    try:
        if opts.verb == 'clear':
            clearpl(node, name) 
        elif opts.verb == 'save':
            savepl(node, fname, timeout, name)
        elif opts.verb == 'load':
            loadpl(node, fname, timeout, name)
        elif opts.verb == 'add':
            loadpl(node, fname, timeout, name, False)
        info('Done.')
    finally:
        node.stop()



    

