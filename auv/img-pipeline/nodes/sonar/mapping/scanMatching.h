/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_SONAR_SCAN_MATCHING_H__
#define __CAUV_SONAR_SCAN_MATCHING_H__

#include <deque>

#include <boost/enable_shared_from_this.hpp>

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/point_cloud.h>

#include <debug/cauv_debug.h>
#include <utility/foreach.h>

#include "common.h"

namespace cauv{
namespace imgproc{

// - Static Utility Functions
static Eigen::Vector3f xythetaFrom4dAffine(Eigen::Matrix4f const& transform){
    // split transform into affine parts:
    const Eigen::Matrix3f rotate    = transform.block<3,3>(0, 0);
    const Eigen::Vector3f translate = transform.block<3,1>(0, 3);

    const Eigen::Vector3f t = rotate*Eigen::Vector3f(1,0,0);
    const float rz = (180/M_PI)*std::atan2(t[1], t[0]);
    return Eigen::Vector3f(translate[0], translate[1], rz);
}

// - Pairwise Matching
class PairwiseMatchException: public std::runtime_error{
    public:
        PairwiseMatchException(std::string const& msg)
            : std::runtime_error(msg){
        }
};

template<typename PointT>
class PairwiseMatcher{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<cloud_t> cloud_ptr;
        typedef boost::shared_ptr<const cloud_t> cloud_const_ptr;
        typedef typename cloud_t::base_cloud_t base_cloud_t;
        typedef typename cloud_t::base_cloud_t::Ptr base_cloud_ptr;

        // - public methods

        /* return confidence (0 - did not match), non-zero, did match, with
         * (0--1] confidence value.
         *
         * The guess is assumed to be in the global coordinate system. It is
         * transformed into 'map's coordinate system:
         *
         * note that:
         * global_point = cloud.relativeTo().relativeTransform() * cloud.relativeTransform() * cloud_point;
         *
         * so:
         * guess * new_cloud_global = map_global
         * guess * (new_cloud.relativeTo().relativeTransform() * new_cloud.relativeTransform() * new_cloud) = map.relativeTransform() * map
         *
         * relative_guess = map.relativeTransform().inverse() * guess * new_cloud.relativeTo().relativeTransform() * new_cloud.relativeTransform()
         *
         * so:
         * guess * new_cloud_local = map_local
         *
         * The returned 'transformation' is in THE MAP's coordinate system -
         * assuming that new_cloud.relativeTo will be set to 'map', but it is
         * the caller's responsibility to do this!
         *
         */
        virtual float transformcloudToMatch(
            cloud_const_ptr map,
            cloud_const_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation,
            base_cloud_ptr& transformed_cloud
        ) const = 0;
};


template <typename PtSrc, typename PtTgt>
class ICP: public pcl::IterativeClosestPoint/*NonLinear*/<PtSrc,PtTgt>{
        typedef pcl::IterativeClosestPoint/*NonLinear*/<PtSrc,PtTgt> base_t;
    public:
        int numIters() const{
            return base_t::nr_iterations_;
        }
        int maxNumIters() const{
            return base_t::max_iterations_;
        }
        float transformationChange() const{
            // should match the check in termination condition of ICP
            return std::fabs((base_t::transformation_ - base_t::previous_transformation_).sum());
        }
        float transformationEpsilon() const{
            return base_t::transformation_epsilon_;
        }
        /*float euclideanFitness() const{
            // should match the check in termination condition of ICP
            return std::fabs(base_t::getFitnessScore(
                base_t::correspondence_distances_,
                base_t::previous_correspondence_distances
            ));
        }
        float euclideanFitnessEpsilon() const{
            return base_t::euclidean_fitness_epsilon_;
        }*/

};


template<typename PointT>
class ICPPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<cloud_t> cloud_ptr;
        typedef boost::shared_ptr<const cloud_t> cloud_const_ptr;        
        typedef typename cloud_t::base_cloud_t base_cloud_t;
        typedef typename cloud_t::base_cloud_t::Ptr base_cloud_ptr;

        ICPPairwiseMatcher(int max_iters,
                           float euclidean_fitness,
                           float transform_eps,
                           float reject_threshold,
                           float max_correspond_dist,
                           float score_thr)
            : PairwiseMatcher<PointT>(),
              m_max_iters(max_iters),
              m_euclidean_fitness(euclidean_fitness),
              m_transform_eps(transform_eps),
              m_reject_threshold(reject_threshold),
              m_max_correspond_dist(max_correspond_dist),
              m_score_thr(score_thr){
        }

        // - public methods
        virtual float transformcloudToMatch(
            cloud_const_ptr map,
            cloud_const_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation,
            base_cloud_ptr& transformed_cloud
        ) const {
            debug() << "ICPPairwiseMatcher" << map->size() << ":" << new_cloud->size() << "points";

            //pcl::IterativeClosestPointNonLinear<PointT,PointT> icp;
            ICP<PointT,PointT> icp;
            icp.setInputCloud(new_cloud);
            icp.setInputTarget(map);

            icp.setMaxCorrespondenceDistance(m_max_correspond_dist);
            icp.setMaximumIterations(m_max_iters);
            icp.setTransformationEpsilon(m_transform_eps);
            icp.setEuclideanFitnessEpsilon(m_euclidean_fitness);
            icp.setRANSACOutlierRejectionThreshold(m_reject_threshold);

            const Eigen::Matrix4f relative_guess = map->relativeTransform().inverse() * guess * new_cloud->globalTransform();

            debug(3) << BashColour::Green << "guess:\n"
                     << guess;
            debug() << BashColour::Green << "relative guess:\n"
                    << relative_guess;

            // do the hard work!
            icp.align(*transformed_cloud, relative_guess);
            // in map's coordinate system:
            const Eigen::Matrix4f final_transform = icp.getFinalTransformation();

            // high is bad (score is sum of squared euclidean distances)
            const float score = icp.getFitnessScore();
            info() << BashColour::Green
                   << "converged:" << icp.hasConverged()
                   << "score:" << score
                   << "after" << icp.numIters() << "/" << m_max_iters
                   << "iterations."
                   << "Transformation change:" << icp.transformationChange()
                   << "/epsilon:" << icp.transformationEpsilon();
                   /*<< "Fitness:" << icp.euclideanFitness()
                   << "/epsilon:" << icp.euclideanFitnessEpsilon();*/

            if(icp.hasConverged() && score < m_score_thr){
                debug() << BashColour::Green << "final transform:\n"
                        << final_transform;
                Eigen::Vector3f xytheta = xythetaFrom4dAffine(final_transform);
                info() << BashColour::Green << "pairwise match:"
                       << xytheta[0] << "," << xytheta[1] << "rot=" << xytheta[2] << "deg";

                transformation = final_transform;
            }else if(score >= m_score_thr){
                info() << BashColour::Brown
                       << "ICP pairwise match failed (error too high: "
                       << score << ">=" << m_score_thr <<")";
                throw PairwiseMatchException("error too high");
            }else{
                info() << BashColour::Red
                       << "ICP pairwise match failed (not converged)";
                throw PairwiseMatchException("failed to converge");
            }

            // TODO: more rigorous match metric
            return 1.0 / (1.0 + score);
        }

    private:
        const int m_max_iters;
        const float m_euclidean_fitness;
        const float m_transform_eps;
        const float m_reject_threshold;
        const float m_max_correspond_dist;
        const float m_score_thr;
};

template<typename PointT>
class NDTPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<cloud_t> cloud_ptr;
        typedef boost::shared_ptr<const cloud_t> cloud_const_ptr;        
        typedef typename cloud_t::base_cloud_t base_cloud_t;
        typedef typename cloud_t::base_cloud_t::Ptr base_cloud_ptr;

        // - public methods
        virtual float transformcloudToMatch(
            cloud_const_ptr map,
            cloud_const_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation,
            base_cloud_ptr& transformed_cloud
        ) const {
            // TODO
            assert(0);
            return 0;
        }
};

} // namespace cauv
} // namespace imgproc


#endif //ndef __CAUV_SONAR_SCAN_MATCHING_H__
