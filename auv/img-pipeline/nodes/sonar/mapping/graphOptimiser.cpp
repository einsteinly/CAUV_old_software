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

#include "graphOptimiser.h"

using namespace cauv;
using namespace cauv::imgproc;

IncrementalPose IncrementalPose::from4dAffine(Eigen::Matrix4f const& a){
    const Eigen::Matrix3f r = a.block<3,3>(0, 0);
    const Eigen::Vector3f t = a.block<3,1>(0, 3);

    const Eigen::Vector3f tmp = r*Eigen::Vector3f(1,0,0);
    const float rz = (180/M_PI)*std::atan2(tmp[1], tmp[0]);
    IncrementalPose incr = {
        t[0], t[1], rz
    };
    return incr;
}

IncrementalPose IncrementalPose::from4dAffineDiff(
    Eigen::Matrix4f const& from,
    Eigen::Matrix4f const& to){
    return from4dAffine(to) -= from4dAffine(from);
}

IncrementalPose& IncrementalPose::operator-=(IncrementalPose const& r){
    dx -= r.dx;
    dy -= r.dy;
    dtheta -= r.dtheta;
    return *this;
}


/* Optimise a constraint graph.
 * !!! paper ref here (Olson et al, Grisetti et al)
 * !!! this could be provided by an external class like scan matching
 *     so that methods can be easily compared.
 */
void GraphOptimiserV1::optimiseGraph(
    constraint_vec const& constraints,
    constraint_vec const& new_constraints
) const{
    
}


