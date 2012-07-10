/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */
#ifndef __CAUV_SSLAM_STUFF_H__
#define __CAUV_SSLAM_STUFF_H__

#include <fstream>

#include <Eigen/Core>

namespace cauv{
struct TimeStamp;

/* difference in floating point seconds */
double operator-(cauv::TimeStamp const& left, cauv::TimeStamp const& right);

/* return [dx, dy, dtheta (about z axis, mathematical angle, radians)] extracted from 4D transformation matrix */
Eigen::Vector3f xyThetaFrom4DAffine(Eigen::Matrix4f const& a);

// will find a home for this later
inline static void saveMat(std::ofstream& f, Eigen::Matrix4f const& m){
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            f.write((char*)&m(i,j), sizeof(m(i,j)));
}

inline static void loadMat(std::ifstream& f, Eigen::Matrix4f& m){
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            f.read((char*)&m(i,j), sizeof(m(i,j)));
}

} // namespace cauv

#endif // ndef __CAUV_SSLAM_STUFF_H__
