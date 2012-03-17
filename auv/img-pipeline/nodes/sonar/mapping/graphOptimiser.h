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
#include <Eigen/Geometry>

namespace cauv{
namespace imgproc{

// - Incremental Pose: as defined in Olson's paper
struct IncrementalPose{
    Eigen::Vector3f x; // [dx, dy, dtheta] (radians)
    
    // have Eigen::Vector3f as member    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float const dx() const{ return x[0]; }
    float const dy() const{ return x[1]; }
    float const detheta() const{ return x[2]; }
    
    IncrementalPose() : x(Eigen::Vector3f::Zero()) { }
    explicit IncrementalPose(Eigen::Vector3f const& v) : x(v) { }
    explicit IncrementalPose(float dx, float dy, float dradians) : x(dx, dy, dradians) { }

    static IncrementalPose from4dAffine(Eigen::Matrix4f const& a);
    static IncrementalPose from4dAffineDiff(Eigen::Matrix4f const& from, Eigen::Matrix4f const& to);

    Eigen::Matrix4f applyTo(Eigen::Matrix4f const& pose) const{
        Eigen::Matrix4f r = pose;
        r.block<2,1>(0,3) += Eigen::Vector2f(x[0], x[1]);
        // !!! TODO: double-check this is the right way around
        r.block<2,2>(0,0) *= Eigen::Matrix2f(Eigen::Rotation2D<float>(x[2]));
        return r;
    }

    inline IncrementalPose& operator-=(IncrementalPose const& r){
        x -= r.x;
        return *this;
    }
    inline IncrementalPose operator-(IncrementalPose const& r) const{
        return IncrementalPose(*this) -= r;
    }

    inline IncrementalPose operator-() const{
        return IncrementalPose(-x);
    }

    inline IncrementalPose& operator+=(IncrementalPose const& r){
        x += r.x;
        return *this;
    }

    inline IncrementalPose& operator*=(float const& scalar){
        x *= scalar;
        return *this;
    }
    inline IncrementalPose operator*(float const& scalar) const{
        return IncrementalPose(*this) *= scalar;
    }
};

/*
struct IncrementalPoseConstraint{
    IncrementalPose a_to_b;
    location_ptr a;
    location_ptr b;
    
    // tag is used as temporary storage for the 'level' of the constraint
    // during graph optimisation
    int tag;

    float weight;
    
    // IncrementalPose has Eigen::Vector3f as member
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    IncrementalPoseConstraint(IncrementalPose const& a_to_b, location_ptr a, location_ptr b)
        : a_to_b(a_to_b), a(a), b(b), tag(0), weight(1){
    }
};
*/

struct RelativePose{
    Eigen::Vector3f x; // [dx, dy, dtheta] (radians)
    
    // have Eigen::Vector3f as member    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    float const dx() const{ return x[0]; }
    float const dy() const{ return x[1]; }
    float const detheta() const{ return x[2]; }
     
    RelativePose() : x(Eigen::Vector3f::Zero()) { }
    explicit RelativePose(Eigen::Vector3f const& v) : x(v) { }
    explicit RelativePose(float dx, float dy, float dradians) : x(dx, dy, dradians) { }

    static RelativePose from4dAffine(Eigen::Matrix4f const& a);
    Eigen::Matrix4f to4dAffine() const;

    Eigen::Matrix4f applyTo(Eigen::Matrix4f const& pose) const{
        return pose * to4dAffine();
    }

};

struct RelativePoseConstraint{
    RelativePose b_wrt_a;
    location_ptr a;
    location_ptr b;

    // tag is used as temporary storage for the 'level' of the constraint
    // during graph optimisation
    int tag;

    float weight;    
    
    // RelativePose has Eigen::Vector3f as member
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    RelativePoseConstraint(RelativePose const& b_wrt_a, location_ptr a, location_ptr b)
        : b_wrt_a(b_wrt_a), a(a), b(b), tag(0), weight(1){
    }
};


// - Graph Optimisation Classes
class GraphOptimiser{
    public:
        /* Optimise a constraint graph. The constraints passed in may be
         * modified, in particular some implementations may find it useful to
         * sort them, but elements MUST NOT be added or removed.
         */
        virtual void optimiseGraph(constraint_vec& constraints,
                                   constraint_vec const& new_constraints) const = 0;
};


/* !!! TODO: more descriptive name
 */
class GraphOptimiserV1: public GraphOptimiser{
    public:
        GraphOptimiserV1(int max_iters)
            : m_max_iters(max_iters){
        }

        /* Optimise a constraint graph.
         * !!! paper ref here (Olson et al, Grisetti et al)
         * !!! this could be provided by an external class like scan matching
         *     so that methods can be easily compared.
         */
        virtual void optimiseGraph(
            constraint_vec& constraints,
            constraint_vec const& new_constraints = constraint_vec()
        ) const;
    
    private:
        int m_max_iters;
};

} // namespace imgproc
} // namespace cauv


#endif // ndef __CAUV_SONAR_SLAM_GRAPHOPT_H__

