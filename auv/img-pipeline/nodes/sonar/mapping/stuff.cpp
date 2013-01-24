/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "stuff.h"

#include <generated/types/TimeStamp.h>

//TODO: merge this into time.h (there's a conflicting definition in control
//which returns milliseconds instead of seconds
double cauv::operator-(cauv::TimeStamp const& left, cauv::TimeStamp const& right){
    const double dsecs = left.secs - right.secs;
    const double dmusecs = left.musecs - right.musecs;
    return dsecs + 1e-6*dmusecs;
}

/* return [dx, dy, dtheta (about z axis, mathematical angle, radians)] extracted from 4D transformation matrix */
Eigen::Vector3f cauv::xyThetaFrom4DAffine(Eigen::Matrix4f const& a){
    const Eigen::Matrix2f r = a.block<2,2>(0, 0);
    const Eigen::Vector2f t = a.block<2,1>(0, 3);

    const Eigen::Vector2f tmp = r * Eigen::Vector2f(1,0);
    const float rz = std::atan2(tmp[1], tmp[0]);
    return Eigen::Vector3f(t[0], t[1], rz);
}

