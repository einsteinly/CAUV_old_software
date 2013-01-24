#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

import fcntl
import traceback
import cPickle as pickle
import os
import time
import functools

class FileLocker(object):
    def __init__(self, fobj):
        self.__f = fobj
    def __enter__(self):
        fcntl.lockf(self.__f, fcntl.LOCK_EX)
        return self
    def __exit__(self, exc_type, exc_val, exc_tb):
        #print exc_type, exc_val, exc_tb
        fcntl.lockf(self.__f, fcntl.LOCK_UN)

def fileCached(cache_duration):
    # decorator generator: cache results of functions with no arguments in a
    # file for cache_duration seconds
    def fileCached_decorator(f):
        @functools.wraps(f)
        def f_once(cachefname='/tmp/co.uk.cauv.%s.pickle' % f.__name__):
            try:
                t = open(cachefname, 'rb+')
            except IOError, e:
                if e.errno == 2:
                    # no such file or directory
                    #print '!!!! opened new cache file'
                    t = open(cachefname, 'wb+')
                else:
                    #print "!!!! couldn't open cache file!"
                    return f()
            with t as cachef:
                # on my system this blocks until the lock is available...  
                with FileLocker(t) as l:
                    try:
                        fcntl.lockf(cachef, fcntl.LOCK_EX)
                        t = None
                        s = cachef.read()
                        if len(s):
                            (r, t) =  pickle.loads(s)
                        tnow = time.time() 
                        if t is not None and tnow - t < cache_duration:
                            #print '!!!! returning cached value'
                            return r
                        #elif t is None:
                        #    print '!!!! no cached value'
                        #else:
                        #    print '!!!! cache is too old'
                        r = f()
                        cachef.truncate(0)
                        cachef.seek(0)
                        cachef.write(pickle.dumps((r, tnow)))
                        cachef.flush()
                        return r
                    except Exception, e:
                        print '!!!! TODO: find out why this exception occured!'
                        traceback.print_exc()
                        return f()
        return f_once
    return fileCached_decorator

