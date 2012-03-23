import math

class vec(object):
    def __new__(cls, *args):
        for vec_class in vec.__subclasses__():
            if vec_class.__name__ == 'vec'+str(len(args))+'d':
                return vec_class(*args)
        return object.__new__(cls, *args)
    def __init__(self, *args):
        self.vec = args
    def __getitem__(self, index):
        return self.vec[index]
    def __setitem__(self, index, value):
        self.vec[index] = value
    def __add__(self, other):
        if len(self.vec)!=len(other.vec):
            return TypeError('Cannot add different size vectors')
        return vec(*[self.vec[x]+other.vec[x] for x in range(len(self.vec))])
    def __sub__(self, other):
        if len(self.vec)!=len(other.vec):
            return TypeError('Cannot subtract different size vectors')
        return vec(*[self.vec[x]-other.vec[x] for x in range(len(self.vec))])
    def __mul__(self, num):
        return vec(*[num*self.vec[x] for x in range(len(self.vec))])
    def __div__(self, num):
        return vec(*[self.vec[x]/num for x in range(len(self.vec))])
    def __abs__(self):
        return math.sqrt(dotProd(self,self))
    def __repr__(self):
        return str(self.vec)
    
class vec2d(vec):
    def __new__(*args):
        return object.__new__(*args)
    def __init__(self,x,y):
        self.vec = [x,y]
    def __setattr__(self, attr, value):
        if attr == 'x':
            self.vec[0] = value
        elif attr == 'y':
            self.vec[1] = value
        else:
            super(vec2d, self).__setattr__(attr, value)
    def __getattr__(self, attr):
        if attr == 'x':
            return self.vec[0]
        elif attr == 'y':
            return self.vec[1]
        else:
            raise AttributeError
    def __repr__(self):
        return "(%f,%f)"%(self.x,self.y)
    
def dotProd(p1,p2):
    if len(p1.vec)!=len(p2.vec):
        return TypeError('Cannot dot product different size vectors')
    return sum([p1.vec[x]*p2.vec[x] for x in range(len(p1.vec))])
    
def crossProd(p1,p2):
    if len(p1.vec)!=2 or len(p2.vec)!=2:
        return TypeError('Cross product only supports 2d vectors at the moment')
    return p1.x*p2.y - p1.y*p2.x