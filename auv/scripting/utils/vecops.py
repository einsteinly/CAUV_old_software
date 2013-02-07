#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from cauv.messaging import floatXYZ

def add(v1, v2):
    v = floatXYZ(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z)
    return v

def sub(v1, v2):
    v = floatXYZ(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z)
    return v

def pow(v1, p):
    return floatXYZ(v1.x**p, v1.y**p, v1.z**p)

def sqr(v1):
    return pow(v1,2)

def sx(v_vec):
    r = floatXYZ(0,0,0)
    for v in v_vec:
        r = add(r, v)
    return r

def sxx(v_vec):
    r = floatXYZ(0,0,0)
    for v in v_vec:
        r = add(r, sqr(v))
    return r

def absv(v):
    return floatXYZ(abs(v.x),abs(v.y),abs(v.z))

def xyz(vec, z = 0):
    for v in vec:
        yield floatXYZ(v.x, v.y, z)
