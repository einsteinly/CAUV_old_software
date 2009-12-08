#ifndef __CAUV_TYPES_H__
#define __CAUV_TYPES_H__

#include <iostream>

using namespace std;

struct floatXYZ {
	float x,y,z;
	floatXYZ(float x = 0, float y = 0, float z = 0);
};

struct floatYPR {
	float yaw, pitch, roll;
	floatYPR(float yaw = 0, float pitch = 0, float roll = 0);
};

struct quaternion {
	float w, x, y, z;
	quaternion(float w = 0, float x = 0, float y = 0, float z = 0);
    quaternion operator*(quaternion q);
    floatYPR to_ypr();
};

struct vector2d {
    float x, y;
    vector2d(float x = 0, float y = 0);
};

ostream& operator<<(ostream& os, const floatYPR& ypr);
ostream& operator<<(ostream& os, const floatXYZ& xyz);
ostream& operator<<(ostream& os, const quaternion& q);
ostream& operator<<(ostream& os, const vector2d& v);

#endif//__CAUV_TYPES_H__
