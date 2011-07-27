# Okay, so some of these aren't really hacks, just things that don't belong
# anywhere else yet.

from sys import float_info
from math import frexp, ldexp

def incFloat(f):
    # increment a floating point number by the smallest possible amount
    # not sure this works with negative numbers
    if f == 0.0:
        return float_info.min
    m, e = frexp(f)
    return ldexp(m + float_info.epsilon / 2, e)


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




