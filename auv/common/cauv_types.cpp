#include "cauv_types.h"
#include <cmath>

floatXYZ::floatXYZ(float x, float y, float z) : x(x), y(y), z(z)
{
}

floatYPR::floatYPR(float yaw, float pitch, float roll) : yaw(yaw), pitch(pitch), roll(roll)
{
}

quaternion::quaternion(float w, float x, float y, float z) : w(w), x(x), y(y), z(z)
{
}
quaternion quaternion::operator*(quaternion q)
{
    quaternion ret(
        w*q.w - x*q.x - y*q.y - z*q.z,
        w*q.x + x*q.w - y*q.z - z*q.y,
        w*q.y - x*q.z + y*q.w + z*q.x,
        w*q.z + x*q.y - y*q.x + z*q.w
    );
    return ret;
}
floatYPR quaternion::to_ypr()
{
    floatYPR ret;

    float sq0 = w*w;
    float sq1 = x*x;
    float sq3 = z*z;

    ret.yaw = atan2(2*x*y + 2*w*z , 2*sq0 + 2*sq1- 1);

    float test = x*z - w*y;
	if (test > 0.499999) { // singularity at north pole
		ret.pitch = M_PI_2;
		ret.roll = 0;
	}
	else if (test < -0.499999) { // singularity at south pole
		ret.pitch = -M_PI_2;
		ret.roll = 0;
	}
    else
    {
        ret.pitch = -asin(2*test);
        ret.roll = atan2(2*y*z + 2*w*x , 2*sq0 + 2*sq3 - 1);
    }
    
    return ret;
}

vector2d::vector2d(float x, float y) : x(x), y(y)
{
}



ostream& operator<<(ostream& os, const floatYPR& ypr)
{
    os << "{ yaw: " << ypr.yaw << ", pitch: " << ypr.pitch << ", roll: " << ypr.roll << " }";
	return os;
}
ostream& operator<<(ostream& os, const floatXYZ& xyz)
{
    os << "{ x: " << xyz.x << ", y: " << xyz.y << ", z: " << xyz.z << " }";
	return os;
}
ostream& operator<<(ostream& os, const quaternion& q)
{
    os << "{ w: " << q.w << ", x: " << q.x << ", y: " << q.y  << ", z: " << q.z << " }";
	return os;
}
ostream& operator<<(ostream& os, const vector2d& v)
{
    os << "{ x: " << v.x << ", y: " << v.y << " }";
	return os;
}
