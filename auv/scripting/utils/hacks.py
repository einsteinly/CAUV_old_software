# Okay, so some of these aren't really hacks, just things that don't belong
# anywhere else yet.

from sys import float_info
from math import frexp, ldexp
from datetime import timedelta

def incFloat(f):
    # increment a floating point number by the smallest possible amount
    # not sure this works with negative numbers
    if f == 0.0:
        return float_info.min
    m, e = frexp(f)
    return ldexp(m + float_info.epsilon / 2, e)

# actually returns a new class resembling the old class with a new base, note
# that r().super(cls, self) won't work, since instances will be instances of r...
# so use super(self.__class__, self) instead
def injectBase(cls, new_base):
    r = type(cls.__name__, (new_base,) + cls.__bases__, dict(cls.__dict__))
    r.__super__ = new_base
    r.__module__ = new_base.__module__
    return r

import functools

# aka memoization, see python-module/cauv/utils.py for file-cached memoization
def once(f):
    @functools.wraps(f)
    def f_once(cache=[]):
        if not len(cache):
            cache.append(f())
        return cache[0]
    return f_once

def hgCmd(cmd):
    import os, shlex, subprocess
    repo_root = '/'.join(
        (os.path.join(os.getcwd(), __file__)).split('/')[:-4]
    )
    hg_cmdstr = 'hg -R %s %s' % (repo_root, cmd)
    dp = subprocess.Popen(shlex.split(hg_cmdstr), stdout = subprocess.PIPE)
    r = dp.communicate()
    return r[0]

@once
def sourceRevision():
    return hgCmd("log -l 1 --template '{node}'")


# Functions for dealing with datetime.timedelta objects:

# So here's a reason to use python 3.2: timedelta objects can finally be
# multiplied / divided and we could get rid of these:
def tdToLongMuSec(td):
    return long(td.microseconds) + td.seconds*1000000L + td.days*86400000000L

def tdToFloatSeconds(td):
    return (long(td.microseconds) + td.seconds*1000000L + td.days*86400000000L)/1000000.0

def tddiv(l, r):
    l_musec = tdToLongMuSec(l)
    r_musec = tdToLongMuSec(r)
    ret = l_musec / float(r_musec)
    #print 'tddiv: %s / %s = %s / %s = %s' % (l, r, l_musec, r_musec, ret)
    return ret

def tdmul(a, b):
    td = None
    scalar = None
    if isinstance(a, timedelta):
        td = a
        scalar = b
    if isinstance(b, timedelta):
        if td is not None:
            raise RuntimeError("you can't multiply two timedeltas")
        td = b
        scalar = a
    return timedelta(microseconds = tdToLongMuSec(td) * scalar)




