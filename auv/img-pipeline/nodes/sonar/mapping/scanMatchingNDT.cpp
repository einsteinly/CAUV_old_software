/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


// hide the subclasses in here to try to split up compiling into chunks that
// take less than 2G RAM...

#include "scanMatching.h"

#include <pcl/registration/ndt_2d.h>

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

template <typename PtSrc, typename PtTgt>
class NDT: public pcl::NormalDistributionsTransform2D<PtSrc,PtTgt>{
        typedef pcl::NormalDistributionsTransform2D<PtSrc,PtTgt> base_t;
    public:
        int numIters() const{
            return base_t::nr_iterations_;
        }
        int maxNumIters() const{
            return base_t::max_iterations_;
        }
        float transformationChange() const{
            // should match the check in termination condition of NDT
            return std::fabs((base_t::transformation_ - base_t::previous_transformation_).sum());
        }
        float transformationEpsilon() const{
            return base_t::transformation_epsilon_;
        }
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


        NDTPairwiseMatcher(int max_iters,
                           float euclidean_fitness,
                           float transform_eps,
                           float max_correspond_dist,
                           float score_thr,
                           float grid_step)
            : PairwiseMatcher<PointT>(),
              m_max_iters(max_iters),
              m_euclidean_fitness(euclidean_fitness),
              m_transform_eps(transform_eps),
              m_max_correspond_dist(max_correspond_dist),
              m_score_thr(score_thr),
              m_grid_step(grid_step){
        }

        // - public methods
        virtual float transformcloudToMatch(
            cloud_const_ptr map,
            cloud_const_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation,
            base_cloud_ptr& transformed_cloud
        ) const {
            debug() << "NDTPairwiseMatcher" << map->size() << ":" << new_cloud->size() << "points";

            NDT<PointT,PointT> ndt;
            ndt.setInputCloud(new_cloud);
            ndt.setInputTarget(map);
            
            const Eigen::Vector3f map_mean = map->mean();
            const Eigen::Matrix3f map_covar = map->covar();
            float sx = std::sqrt(map_covar(0,0));
            float sy = std::sqrt(map_covar(1,1));
            debug() << "map cloud mean:" << map->mean().transpose()
                    << ", covar:\n" << map_covar;
            debug() << "map sx, sy:" << sx << sy;

            ndt.setMaxCorrespondenceDistance(m_max_correspond_dist); // for getFitnessScore only
            ndt.setGridCentre(Eigen::Vector2f(map_mean[0], map_mean[1]));
            ndt.setGridExtent(Eigen::Vector2f(3*sx, 3*sy));
            ndt.setGridStep(Eigen::Vector2f(m_grid_step,m_grid_step));
            ndt.setOptimizationStepSize(Eigen::Vector3d(0.05,0.05,0.05));
            ndt.setMaximumIterations(m_max_iters);
            ndt.setTransformationEpsilon(m_transform_eps);
            ndt.setEuclideanFitnessEpsilon(m_euclidean_fitness);

            const Eigen::Matrix4f relative_guess = map->relativeTransform().inverse() * guess * new_cloud->globalTransform();

            debug(3) << BashColour::Green << "guess:\n"
                     << guess;
            debug() << BashColour::Green << "relative guess:\n"
                    << relative_guess;

            // do the hard work!
            ndt.align(*transformed_cloud, relative_guess);
            // in map's coordinate system:
            const Eigen::Matrix4f final_transform = ndt.getFinalTransformation();
            
            // if the alignment really diverged, getFitnessScore doesn't even
            // work:
            if(!(final_transform == final_transform)){
                error() << "NaN transform!\n" << final_transform;
                throw PairwiseMatchException("NaN transform!");
            }

            // high is bad (score is sum of squared euclidean distances)
            const float score = ndt.getFitnessScore(m_max_correspond_dist);
            info() << BashColour::Green
                   << "converged:" << ndt.hasConverged()
                   << "score:" << score
                   << "after" << ndt.numIters() << "/" << m_max_iters
                   << "iterations."
                   << "Transformation change:" << ndt.transformationChange()
                   << "/epsilon:" << ndt.transformationEpsilon();

            if(ndt.hasConverged() && score < m_score_thr){
                debug() << BashColour::Green << "final transform:\n"
                        << final_transform;
                Eigen::Vector3f xytheta = xythetaFrom4dAffine(final_transform);
                info() << BashColour::Green << "pairwise match:"
                       << xytheta[0] << "," << xytheta[1] << "rot=" << xytheta[2] << "deg";
                // transform is map->new cloud, without inverse it's new->map
                transformation = final_transform;
            }else if(score >= m_score_thr){
                info() << BashColour::Brown
                       << "NDT pairwise match failed (error too high: "
                       << score << ">=" << m_score_thr <<")";
                throw PairwiseMatchException("error too high");
            }else{
                info() << BashColour::Red
                       << "NDT pairwise match failed (not converged)";
                throw PairwiseMatchException("failed to converge");
            }

            // TODO: more rigorous match metric
            return 1.0 / (1.0 + score);
        }
        
    private:
        const int m_max_iters;
        const float m_euclidean_fitness;
        const float m_transform_eps;
        const float m_max_correspond_dist;
        const float m_score_thr;
        const float m_grid_step;
};


// - make_shared functions:

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeNDTPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float max_correspond_dist,
    float score_thr,
    float grid_step
){
    return boost::make_shared< NDTPairwiseMatcher<pcl::PointXYZ> >(
        max_iters, euclidean_fitness, transform_eps, max_correspond_dist, score_thr, grid_step
    );
}

} // namespace cauv
} // namespace imgproc


