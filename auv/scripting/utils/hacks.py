from sys import float_info
from math import frexp, ldexp

def incFloat(f):
    # increment a floating point number by the smallest possible amount
    # not sure this works with negative numbers
    if f == 0.0:
        return float_info.min
    m, e = frexp(f)
    return ldexp(m + float_info.epsilon / 2, e)

