#
# This module defines a dictionary type that will interpolate between values
# for keys that do not exist in the dictionary. Two interpolation methods
# (zero-order hold, and linear) are provided.
#

# Standard Library
import bisect
from datetime import timedelta

# 3rd Party
import blist           # BSD license

# CAUV
from hacks import tddiv, tdmul

def linearInterp(xlow, ylow, xhi, yhi, x):
    # TODO: handle xhi = xlo gracefully
    if x == xlow:
        r = ylow
    elif x == xhi:
        r = yhi
    else:
        n = ((yhi - ylow) * (x - xlow))
        d = (xhi - xlow)
        if type(n) == timedelta:
            # silly datetime.timedelta doesn't support division
            r = ylow + tddiv(n, d)
        else:
            r = ylow + n / d
    #print 'low: %s %s\n  x: %s\n hi: %s %s\n  -> %s' % (xlow, ylow, x, xhi, yhi, r)
    return r

def linearInterp_timedeltas(xlow, ylow, xhi, yhi, x):
    # TODO: handle xhi = xlo gracefully
    if x == xlow:
        r = ylow
    elif x == xhi:
        r = yhi
    else:
        n = ((yhi - ylow) * (x - xlow))
        d = (xhi - xlow)
        # silly datetime.timedelta doesn't support division
        r = ylow + tddiv(n, d)
    #print 'low: %s %s\n  x: %s\n hi: %s %s\n  -> %s' % (xlow, ylow, x, xhi, yhi, r)
    return r

def zeroOrderInterp(xlo, ylo, xhi, yhi, x):
    return ylo

class OutOfRange_Low(KeyError):
    def __init__(self, s):
        KeyError.__init__(self, s)

class OutOfRange_High(KeyError):
    def __init__(self, s):
        KeyError.__init__(self, s)

class PiecewiseApprox(blist.sorteddict):
    def __init__(self, interp, rfunc=round):
        blist.sorteddict.__init__(self)
        self.interpolate = interp
        self.rfunc = rfunc

    def __getitem__(self, k):
        # interpolates if a value is not present for the specified key
        # NB: self._sortedkeys not copied! (faster than using self.keys())
        sorted_keys = self._sortedkeys #pylint: disable=E1101
        
        # print '__getitem__:\n', '\n'.join(map(str,sorted_keys)), 'k=', k
        ilow = bisect.bisect(sorted_keys, k) - 1
        if ilow < 0:
            raise OutOfRange_Low('out of range: lt')
        klow = sorted_keys[ilow]

        ihi = ilow + 1 
        try:
            khi = sorted_keys[ihi]
        except IndexError:
            if sorted_keys[ilow] != k and self.interpolate is not zeroOrderInterp:
                raise OutOfRange_High('out of range: gt')
            else:
                return self.rfunc(self._map[klow])

        '''#dbg
        for i, y in enumerate(sorted_keys):
            if i == ilow:
                print i, y, '<-- low'
                print '=', k
            elif i == ihi:
                print i, y, '<-- hi'
            else:
                print i, y#'''
        r = self.interpolate(
            klow, self._map[klow],
            khi, self._map[khi],
            k
        )
        return self.rfunc(r)

class ZeroOrderPiecewiseApprox(blist.sorteddict):
    def __init__(self):
        blist.sorteddict.__init__(self)
    
    def __getitem__(self, k):
        # interpolates if a value is not present for the specified key
        # NB: self._sortedkeys not copied! (faster than using self.keys())
        sorted_keys = self._sortedkeys #pylint: disable=E1101
        
        # print '__getitem__:\n', '\n'.join(map(str,sorted_keys)), 'k=', k
        ilow = bisect.bisect(sorted_keys, k) - 1
        if ilow < 0:
            raise OutOfRange_Low('out of range: lt')
        klow = sorted_keys[ilow]

        return self._map[klow]

