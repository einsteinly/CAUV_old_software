#!/usr/bin/env python2.7

#from __future__ import with_satement

import cauv.node
import cauv.messaging as messaging
import cauv.pipeline as pipeline
from cauv.debug import debug, warning, error, info
try:
    import cauv.yamlpipe
except ImportError as e:
    error("Failed to import yamlpipe, only pickle load/save availible")
import cauv.picklepipe

import time
import argparse

def savepl(node, fname, timeout=3.0, name='default'):
    with open(fname, 'wb') as outf:
        info('Initializing pipeline model (%s)...' % name)
        model = pipeline.Model(node, name)

        info('Getting pipeline state...')
        saved = model.get(timeout)
        
        info('Pickling...')
        if fname.endswith('.pipe'):
            cauv.picklepipe.dump(outf, saved)
        else:
            cauv.yamlpipe.dump(outf, saved)


def loadpl(node, fname, name='default'):
    with open(fname, 'rb') as inf:
        info('Initializing pipeline model (%s)...' % name)
        model = pipeline.Model(node, name)

        info('Loading...')
        if fname.endswith('.pipe'):
            saved = cauv.picklepipe.load(inf)
        else:
            saved = cauv.yamlpipe.load(inf)
        
        info('Setting pipeline state...')
        model.set(saved)

def clearpl(node, name='default'):
    info('Initializing pipeline model (%s)...' % name)
    model = pipeline.Model(node, name)

    info('Clearing...')
    model.clear()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", "--pipeline-name", default="default")
    parser.add_argument("-t", "--timeout", type=float, default=3.0)
    parser.add_argument("verb", choices=('load', 'save', 'clear'))
    parser.add_argument("file", help='pipeline file')

    opts, unknown_args = parser.parse_known_args()
    fname = opts.file
    timeout = opts.timeout
    name = opts.pipeline_name
    
    info('Connecting...')
    node = cauv.node.Node("py-plsave", unknown_args)
    try:
        if opts.verb == 'clear':
            clearpl(node, name) 
        elif opts.verb == 'save':
            savepl(node, fname, timeout, name)
        elif opts.verb == 'load':
            loadpl(node, fname, name)
        info('Done.')
    except:
        import traceback
        error(traceback.format_exc())
    finally:
        node.stop()
