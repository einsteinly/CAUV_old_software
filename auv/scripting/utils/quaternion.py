''' The CAUV Quaternion Library: because We Couldn't Find One That Worked'''

# Standard Library
import math
from math import sin, cos

Very_Small_Value = 1e-7


class Vec3(object):
    def __init__(self, x, y, z):
        '''Tiny helper class that implements cross product, etc.'''
        super(Vec3, self).__init__()
        self.x = x
        self.y = y
        self.z = z
    def __mul__(self, scalar):
        if isinstance(scalar, Vec3):
            raise Exception('Use .cross and .dot for vector products')
        return Vec3(self.x*scalar, self.y*scalar, self.z*scalar)
    def __add__(self, other):
        return Vec3(self.x + other.x, self.y + other.y, self.z + other.z)
    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z
    def cross(self, other):
        return Vec3(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    def __len__(self):
        return 3
    def __getitem__(self, k):
        return (self.x, self.y, self.z)[k]
    def __setitem__(self, k, v):
        if k == 0:
            self.x = v
        elif k == 1:
            self.y = v
        elif k == 2:
            self.z = v
        else:
            raise IndexError()
    def __iter__(self):
        for i in (0, 1, 2):
            yield self[i]
    def __repr__(self):
        return '(%g,%g,%g)' % (self.x, self.y, self.z)


class Quaternion(object):
    def __init__(self, q0, q1, q2, q3):
        '''Create a Quaternion from component values.
            
            Classmethod constructors are provided for clarity and ease of use:
                fromEuler(xyz)
                fromAngleAxis(xyz_axis, theta_radians)
                identity()

        '''
        super(Quaternion, self).__init__()
        self.q3 = q3 # this is the real part, when viewed as a complex number
        self.q0 = q0 # imaginary x
        self.q1 = q1 # imaginary y
        self.q2 = q2 # imaginary z
    
    @classmethod
    def fromEuler(cls, xyz):
        '''Create a Quaternion directly from rotations about the x, y and z axes: the rotations are applied in order X, then Y, then Z.'''
        cos_x_2 = cos(xyz[0] / 2.0)
        cos_y_2 = cos(xyz[1] / 2.0)
        cos_z_2 = cos(xyz[2] / 2.0)
        sin_x_2 = sin(xyz[0] / 2.0)
        sin_y_2 = sin(xyz[1] / 2.0)
        sin_z_2 = sin(xyz[2] / 2.0)
        return cls(sin_x_2*cos_y_2*cos_z_2 - cos_x_2*sin_y_2*sin_z_2,
                   cos_x_2*sin_y_2*cos_z_2 + sin_x_2*cos_y_2*sin_z_2,
                   cos_x_2*cos_y_2*sin_z_2 - sin_x_2*sin_y_2*cos_z_2,
                   cos_x_2*cos_y_2*cos_z_2 + sin_x_2*sin_y_2*sin_z_2)

    @classmethod
    def fromAngleAxis(cls, xyz_axis, theta_radians):
        '''Create Quaternion from a rotation theta_radians about axis xyz_axis (right hand rule)'''
        sin_t2 = sin(theta_radians/2.0)
        cos_t2 = cos(theta_radians/2.0)
        return cls(sin_t2 * xyz_axis[0],
                   sin_t2 * xyz_axis[1],
                   sin_t2 * xyz_axis[2],
                   cos_t2)
    
    @classmethod
    def identity(cls):
        '''Create one of the (many) identity quaternions. If you care about which one, use fromAngleAxis(axis, 0)'''
        return cls(0,0,0,1)
    

    def real(self):
        '''Return the real part of the quaternion.'''
        return self.q3

    def imaginary(self):
        '''Return the three imaginary parts of the quaternion.'''
        return Vec3(self.q0, self.q1, self.q2)
    
    def angle(self):
        '''Return the angle part of the angle-axis rotation.'''
        return 2*math.acos(self.q3)
    
    def axis(self):
        '''Return the axis part of the angle-axis rotation.'''
        if self.angle() > Very_Small_Value:
            scale = 1.0 / sin(self.angle()/2.0)
            return Vec3(self.q0*scale, self.q1*scale, self.a2*scale)
        else:
            return Vec3(1, 0, 0)

    def conjugate(self):
        '''Return the complex conjugate of this quaternion.'''
        return Quaternion(-self.q0, -self.q1, -self.q2, self.q3)

    def inverse(self):
        '''Return the inverse of this quaternion.
            
            if
                B = q1.rotate(A)
            then
                A = q1.inverse().rotate(B)
        '''
        return self.conjugate() / self.sxx()

    def normalised(self):
        '''Return a normalised version of this quaternion.'''
        m = 1.0 / sqrt(self.sxx())
        return Quaternion(m*self.q0, m*self.q1, m*self.q2, m*self.q3)

    def minimum(self):
        '''Return the minimum equivalent rotation.
            
            The returned rotation is about the same axis but with angle in the
            range [-pi,pi].
        '''
        new_angle = self.angle()
        if new_angle > math.pi:
            new_angle -= (floor(new_angle/(math.pi*2))+1) * math.pi * 2
        elif new_angle < -math.pi:
            new_angle += (floor(-new_angle/(math.pi*2))+1) * math.pi * 2
        return Quaternion.fromAngleAxis(new_angle, self.axis())

    def sxx(self):
        '''Return the sum-squared of the quaternion elements.'''
        return self.q0*self.q0 + self.q1*self.q1 + self.q2*self.q2 + self.q3*self.q3

    def rotate(self, v, vec_construct=Vec3):
        '''Rotate xyz vector v, returning a newly created vector with vec_construct'''
        # yes this optimisation is worthwhile
        q0, q1, q2, q3 = (self.q0, self.q1, self.q2, self.q3)
        v0, v1, v2 = (v[0], v[1], v[2])
        t0 =  q3 * q0
        t1 =  q3 * q1
        t2 =  q3 * q2
        t3 = -q0 * q0
        t4 =  q0 * q1
        t5 =  q0 * q2
        t6 = -q1 * q1
        t7 =  q1 * q2
        t8 = -q2 * q2
        d = vec_construct((t6 + t8) * v0 + (t4 - t2) * v1 + (t1 + t5) * v2,
                          (t2 + t4) * v0 + (t3 + t8) * v1 + (t7 - t0) * v2,
                          (t5 - t1) * v0 + (t0 + t7) * v1 + (t3 + t6) * v2)
        return v + (d * 2)

    def __add__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                self.q0 + other.q0,
                self.q1 + other.q1,
                self.q2 + other.q2,
                self.q3 + other.q3
            )
        else:
            return NotImplemented
    def __sub__(self, other):
        if isinstance(other, Quaternion):
            return Quaternion(
                self.q0 - other.q0,
                self.q1 - other.q1,
                self.q2 - other.q2,
                self.q3 - other.q3
            )
        else:
            return NotImplemented
    def __mul__(self, other):
        if isinstance(other, Quaternion):
            si = self.imaginary()
            oi = other.imaginary()
            real = self.real() * other.real() - si.dot(oi)
            imag = si.cross(oi) + si * other.real() + oi * self.real()
            return Quaternion(imag[0], imag[1], imag[2], real)
        else:
            return Quaternion(self.q0*other, self.q1*other, self.q2*other, self.q3*other)
    def __imul__(self, other):
        if isinstance(other, Quaternion):
            si = self.imaginary()
            oi = other.imaginary()
            real = self.real() * other.real() - si.dot(oi)
            imag = si.cross(oi) + si * other.real() + oi * self.real()
            self.q0 = imag[0]
            self.q1 = imag[1]
            self.q2 = imag[2]
            self.q3 = real
            return self
        else:
            self.q0 *= other
            self.q1 *= other
            self.q2 *= other
            self.q3 *= other
            return self
    def __div__(self, other):
        if isinstance(other, Quaternion):
            return NotImplemented
        else:
            return Quaternion(self.q0/other, self.q1/other, self.q2/other, self.q3/other)
    def __truediv__(self, other):
        if isinstance(other, Quaternion):
            return NotImplemented
        else:
            return Quaternion(self.q0/other, self.q1/other, self.q2/other, self.q3/other)
    def __repr__(self):
        return '(%g,%g,%g)' % (self.x, self.y, self.z)


def test():
    def assert_near(a, b):
        if abs(a-b) > 0.001:
            raise AssertionError('%s != %s' % (a, b))
    
    q1 = Quaternion.fromEuler((math.radians(45), 0, 0))
    r = q1.rotate(Vec3(1, 0, 0))
    assert_near(r[0], 1); assert_near(r[1], 0); assert_near(r[2], 0)
    r = q1.rotate(Vec3(0, 1, 0))
    assert_near(r[0], 0); assert_near(r[1], (2**0.5)/2); assert_near(r[2], (2**0.5)/2)
    r = q1.rotate(Vec3(0, 0, 1))
    assert_near(r[0], 0); assert_near(r[1], -(2**0.5)/2); assert_near(r[2], (2**0.5)/2)

    q2 = Quaternion.fromEuler((0, math.radians(45), 0))
    r = q2.rotate(Vec3(1, 0, 0))
    assert_near(r[0], (2**0.5)/2); assert_near(r[1], 0); assert_near(r[2], -(2**0.5)/2)
    r = q2.rotate(Vec3(0, 1, 0))
    assert_near(r[0], 0); assert_near(r[1], 1); assert_near(r[2], 0)
    r = q2.rotate(Vec3(0, 0, 1))
    assert_near(r[0], (2**0.5)/2); assert_near(r[1], 0); assert_near(r[2], (2**0.5)/2)

    q3 = Quaternion.fromEuler((0, 0, math.radians(45)))

    r = q3.rotate(Vec3(1, 0, 0))
    assert_near(r[0], (2**0.5)/2); assert_near(r[1], (2**0.5)/2); assert_near(r[2], 0)
    r = q3.rotate(Vec3(0, 1, 0))
    assert_near(r[0], -(2**0.5)/2); assert_near(r[1], (2**0.5)/2); assert_near(r[2], 0)
    r = q3.rotate(Vec3(0, 0, 1))
    assert_near(r[0], 0); assert_near(r[1], 0); assert_near(r[2], 1)

    for v in (Vec3(1, 0, 0), Vec3(0, 1, 0), Vec3(0, 0, 1), Vec3(1, 1, 0), Vec3(-1, 0, 1)):
        a = q2.rotate(q1.rotate(v))
        b = (q2 * q1).rotate(v)
        assert_near(a[0],b[0]); assert_near(a[1],b[1]); assert_near(a[2],b[2])

        a = q3.rotate(q1.rotate(v))
        b = (q3 * q1).rotate(v)
        assert_near(a[0],b[0]); assert_near(a[1],b[1]); assert_near(a[2],b[2])

        a = q2.rotate(q3.rotate(v))
        b = (q2 * q3).rotate(v)
        assert_near(a[0],b[0]); assert_near(a[1],b[1]); assert_near(a[2],b[2])
