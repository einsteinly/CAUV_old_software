#
# Copyright 2013 Cambridge Hydronautics Ltd.
#
# See license.txt for details.
#

from cauv import messaging

class BoundedBase(object):
    def __new__(self, value):
        if self.lower<=value and value<=self.upper: # pylint: disable=E1101
            return value
        elif self.wraps: #pylint: disable=E1101
            #set move everything so lower bound is 0, mod interval size, then add lower to get back
            return ((value-self.lower)%(self.upper-self.lower))+self.lower # pylint: disable=E1101
        else:
            raise TypeError('Value set exceeded clamped bounds')
    @classmethod
    def asParamValue(cls,value):
        if cls.wraps: # pylint: disable=E1101
            bound_type = messaging.BoundedFloatType.Wraps
        else:
            bound_type = messaging.BoundedFloatType.Clamps
        return messaging.BoundedFloat(value, cls.lower, cls.upper, bound_type) # pylint: disable=E1101

class BoundedFloatBase(BoundedBase):
    def __new__(self, value):
        return BoundedBase.__new__(self, float(value))
class BoundedIntBase(BoundedBase):
    def __new__(self, value):
        return BoundedBase.__new__(self, int(value))

def BoundFloat(lower, upper, wraps):
    return type('BoundedFloat', (BoundedFloatBase, ), {'lower':lower,'upper':upper,'wraps':wraps})

def BoundInt(lower, upper, wraps):
    return type('BoundedInt', (BoundedIntBase, ), {'lower':lower,'upper':upper,'wraps':wraps})
            
MotorValue = BoundInt(-128,128,False)
