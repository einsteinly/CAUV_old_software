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

#ifndef __CAUV_SONAR_SLAM_GRAPHOPT_H__
#define __CAUV_SONAR_SLAM_GRAPHOPT_H__

#include "common.h"

#include <Eigen/Core>

namespace cauv{
namespace imgproc{

// - Incremental Pose: as defined in Olson's paper
struct IncrementalPose{
    float dx;
    float dy;
    float dtheta;

    static IncrementalPose from4dAffine(Eigen::Matrix4f const& a);
    static IncrementalPose from4dAffineDiff(Eigen::Matrix4f const& from, Eigen::Matrix4f const& to);
    IncrementalPose& operator-=(IncrementalPose const& r);
};

struct IncrementalPoseConstraint{
    IncrementalPose a_to_b;
    location_ptr a;
    location_ptr b;

    IncrementalPoseConstraint(IncrementalPose const& a_to_b, location_ptr a, location_ptr b)
        : a_to_b(a_to_b), a(a), b(b){
    }
};


// - Graph Optimisation Classes
class GraphOptimiser{
    public:
        /* Optimise a constraint graph.
         */
        virtual void optimiseGraph(constraint_vec const& constraints,
                                   constraint_vec const& new_constraints) const = 0;
};


/* !!! TODO: more descriptive name
 */
class GraphOptimiserV1: public GraphOptimiser{
    public:
        /* Optimise a constraint graph.
         * !!! paper ref here (Olson et al, Grisetti et al)
         * !!! this could be provided by an external class like scan matching
         *     so that methods can be easily compared.
         */
        virtual void optimiseGraph(
            constraint_vec const& constraints,
            constraint_vec const& new_constraints = constraint_vec()
        ) const;
};

} // namespace imgproc
} // namespace cauv


#endif // ndef __CAUV_SONAR_SLAM_GRAPHOPT_H__

