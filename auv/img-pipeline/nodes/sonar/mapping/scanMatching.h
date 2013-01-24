/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_SONAR_SCAN_MATCHING_H__
#define __CAUV_SONAR_SCAN_MATCHING_H__

#include <boost/shared_ptr.hpp>

#include <pcl/point_cloud.h>

#include "slamCloudPart.h"

namespace cauv{
namespace imgproc{

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
        virtual ~PairwiseMatcher(){ }

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


// - functions to instantiate (opaque) scan matcher subclasses
// (subclasses are defined in scanMatching.cpp)

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeICPPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float reject_threshold,
    float max_correspond_dist,
    float score_thr,
    int ransac_iters
);

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeICPNonLinearPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float reject_threshold,
    float max_correspond_dist,
    float score_thr,
    int ransac_iters
);

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeNDTPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float max_correspond_dist,
    float score_thr,
    float grid_step
);

} // namespace cauv
} // namespace imgproc


#endif //ndef __CAUV_SONAR_SCAN_MATCHING_H__
