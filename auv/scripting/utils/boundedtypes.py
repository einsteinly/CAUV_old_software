from cauv import messaging


class BoundedFloatBase(object):
    def __new__(self, value):
        value=float(value)
        if self.lower<=value and value<=self.upper:
            return value
        elif self.wraps:
            #set move everything so lower bound is 0, mod interval size, then add lower to get back
            return ((value-self.lower)%(self.upper-self.lower))+self.lower
        else:
            raise TypeError('Value set exceeded clamped bounds')
    def asParamValue(self, value):
        if self.wraps:
            bound_type = messaging.BoundedFloatType.Wraps
        else:
            bound_type = messaging.BoundedFloatType.Clamps
        return messaging.BoundedFloat(self.value, self.lower, self.upper, bound_type)
        
def BoundFloat(lower, upper, wraps):
    return type('BoundedFloat', (BoundedFloatBase, ), {'lower':lower,'upper':upper,'wraps':wraps})

class BoundedIntBase(object):
    def __new__(self, value):
        value=int(value)
        if self.lower<=value and value<=self.upper:
            return value
        elif self.wraps:
            #set move everything so lower bound is 0, mod interval size, then add lower to get back
            return ((value-self.lower)%(self.upper-self.lower))+self.lower
        else:
            raise TypeError('Value set exceeded clamped bounds')

def BoundInt(lower, upper, wraps):
    return type('Boundedint', (BoundedIntBase, ), {'lower':lower,'upper':upper,'wraps':wraps})
            
MotorValue = BoundInt(-128,128,False)