/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


// hide the subclasses in here to try to split up compiling into chunks that
// take less than 2G RAM...

#include "scanMatching.h"

#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/warp_point_rigid_3d.h>
//#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation.h>

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>

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

// !!! TODO: this might be neater and more efficient if applied as a
// CorrespondenceEstimation class, rather than transformation estimation...

template<typename PointT>
class TransformationEstimationCropping: public pcl::registration::TransformationEstimation<PointT, PointT>{
    public:
        typedef pcl::registration::TransformationEstimation<PointT, PointT> TransformationEstimation;
        typedef typename TransformationEstimation::Ptr TransformationEstimationPtr;
        typedef std::vector<int> Indices;
        typedef boost::shared_ptr<Indices> IndicesPtr;
            
        typedef pcl::PointCloud<PointT> PointCloud;
        typedef typename pcl::PointCloud<PointT>::Ptr PointCloudPtr;

        TransformationEstimationCropping(
            TransformationEstimationPtr delegate,
            typename pcl::PointCloud<PointT>::ConstPtr target_cloud_ptr
        )   : pcl::registration::TransformationEstimation<PointT, PointT>(),
              m_delegate(delegate),
              m_hull_alpha(1e3f),
              m_tgt_cloud(target_cloud_ptr),
              m_tgt_hull_cloud(),
              m_tgt_hull_polygons(),
              m_dim(0){
            pcl::ConcaveHull<PointT> hull_calculator;
            m_tgt_hull_cloud.reset(new pcl::PointCloud<PointT>);

            hull_calculator.setInputCloud(m_tgt_cloud);
            hull_calculator.setAlpha(m_hull_alpha);
            hull_calculator.reconstruct(*m_tgt_hull_cloud, m_tgt_hull_polygons);
            m_dim = hull_calculator.getDim();
        }

        virtual ~TransformationEstimationCropping(){
        }
        
        static void nullCloudDeleter(PointCloud const*){}
        static void nullIndicesDeleter(Indices const*){}

        // this is the one ICP uses, so it's the one we need to reimplement
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointT> &cloud_src,
            const Indices &indices_src,
            const pcl::PointCloud<PointT> &cloud_tgt,
            const Indices &indices_tgt,
            Eigen::Matrix4f &transformation_matrix){

            if(indices_src.size() != indices_tgt.size()){
                error() << "estimateRigidTransformation: correspondences are not 1:1!";
                return;
            }

            pcl::CropHull<PointT> crop_filter; 

            crop_filter.setInputCloud(PointCloudPtr(const_cast<PointCloud*>(&cloud_src), &nullCloudDeleter)); // yuck
            crop_filter.setIndices(IndicesPtr(const_cast<Indices*>(&indices_src), &nullIndicesDeleter));      // eww
            crop_filter.setHullCloud(m_tgt_hull_cloud);
            crop_filter.setHullIndices(m_tgt_hull_polygons);
            crop_filter.setDim(m_dim);
            
            Indices filtered_indices_tgt;
            Indices filtered_indices_src;

            crop_filter.filter(filtered_indices_src);

            filtered_indices_tgt.reserve(filtered_indices_src.size());
            
            //debug() << "cropping" << indices_src.size() << " -> "
            //        << filtered_indices_src.size() << "correspondences";

            // filter tgt indices correspondingly:
            std::size_t i = 0, filtered_i = 0;
            for(filtered_i = 0; filtered_i < filtered_indices_src.size(); filtered_i++){
                while(indices_src[i] != filtered_indices_src[filtered_i] && i < indices_src.size())
                    i++;
                if(i != indices_src.size())
                    filtered_indices_tgt.push_back(indices_tgt[i]);
            }

            m_delegate->estimateRigidTransformation(
                cloud_src, filtered_indices_src,
                cloud_tgt, filtered_indices_tgt,
                transformation_matrix
            );
        }

    protected:
        // don't care about others, make sure people can't use them
        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointT> &,
            const pcl::PointCloud<PointT> &,
            Eigen::Matrix4f &){
        }

        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointT> &,
            const Indices &,
            const pcl::PointCloud<PointT> &,
            Eigen::Matrix4f &){
        }

        virtual void
        estimateRigidTransformation (
            const pcl::PointCloud<PointT> &,
            const pcl::PointCloud<PointT> &,
            const pcl::Correspondences &,
            Eigen::Matrix4f &){
        }
    
    private:
        TransformationEstimationPtr m_delegate;
        float m_hull_alpha;
        typename pcl::PointCloud<PointT>::ConstPtr m_tgt_cloud;
        typename pcl::PointCloud<PointT>::Ptr m_tgt_hull_cloud;
        std::vector<pcl::Vertices> m_tgt_hull_polygons;
        int m_dim;
};

// Valid BaseT are pcl::IterativeClosestPoint and // pcl::IterativeClosestPointNonLinear
template <typename BaseT>
class ICP: public BaseT{
        typedef BaseT base_t;
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
        typename base_t::TransformationEstimationPtr getTransformationEstimation() const{
            return base_t::transformation_estimation_;
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

template<typename PointT, typename BaseT>
class _ICPPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<cloud_t> cloud_ptr;
        typedef boost::shared_ptr<const cloud_t> cloud_const_ptr;        
        typedef typename cloud_t::base_cloud_t base_cloud_t;
        typedef typename cloud_t::base_cloud_t::Ptr base_cloud_ptr;

        _ICPPairwiseMatcher(int max_iters,
                           float euclidean_fitness,
                           float transform_eps,
                           float reject_threshold,
                           float max_correspond_dist,
                           float score_thr,
                           int ransac_iters)
            : PairwiseMatcher<PointT>(),
              m_max_iters(max_iters),
              m_euclidean_fitness(euclidean_fitness),
              m_transform_eps(transform_eps),
              m_reject_threshold(reject_threshold),
              m_max_correspond_dist(max_correspond_dist),
              m_score_thr(score_thr),
              m_ransac_iters(ransac_iters){
        }

        // - public methods
        virtual float transformcloudToMatch(
            cloud_const_ptr map,
            cloud_const_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation,
            base_cloud_ptr& transformed_cloud
        ) const {
            //debug() << "ICPPairwiseMatcher" << map->size() << ":" << new_cloud->size() << "points";

            ICP<BaseT> icp;
            icp.setInputCloud(new_cloud);
            icp.setInputTarget(map);
            
            // Rigid 3D motion (x,y,rotation):
            /*boost::shared_ptr< pcl::WarpPointRigid3D<PointT, PointT> > warp_fn(
                new pcl::WarpPointRigid3D<PointT, PointT>
            );
            // Levenberg Marquardt estimation
            boost::shared_ptr< pcl::registration::TransformationEstimationLM<PointT, PointT> > te(
                new pcl::registration::TransformationEstimationLM<PointT, PointT>
            );
            te->setWarpFunction(warp_fn);

            icp.setTransformationEstimation(te);*/
            
            // don't use correspondences to points outside the map cloud
            typename ICP<BaseT>::TransformationEstimationPtr te(
                new TransformationEstimationCropping<PointT>(icp.getTransformationEstimation(), map)
            );
            icp.setTransformationEstimation(te);

            icp.setMaxCorrespondenceDistance(m_max_correspond_dist);
            icp.setMaximumIterations(m_max_iters);
            icp.setTransformationEpsilon(m_transform_eps);
            icp.setEuclideanFitnessEpsilon(m_euclidean_fitness);
            icp.setRANSACOutlierRejectionThreshold(m_reject_threshold);
            //debug() << "default ransac iters: " << icp.getRANSACIterations() << "will set to:" << m_ransac_iters;
            icp.setRANSACIterations(m_ransac_iters);

            const Eigen::Matrix4f relative_guess = map->relativeTransform().inverse() * guess * new_cloud->globalTransform();

            /*debug(3) << BashColour::Green << "guess:\n"
                     << guess;
            debug(2) << BashColour::Green << "relative guess:\n"
                     << relative_guess;*/

            // do the hard work!
            icp.align(*transformed_cloud, relative_guess);
            // in map's coordinate system:
            const Eigen::Matrix4f final_transform = icp.getFinalTransformation();

            // if the alignment really diverged, getFitnessScore doesn't even
            // work:
            if(!(final_transform == final_transform)){
                error() << "NaN transform!\n" << final_transform;
                throw PairwiseMatchException("NaN transform!");
            }

            // high is bad (score is sum of squared euclidean distances)
            const float score = icp.getFitnessScore(m_max_correspond_dist);
            info infostream;
            infostream << BashColour::Green
                       << "ICP converged:" << icp.hasConverged()
                       << "score:" << score
                       << "after" << icp.numIters() << "/" << m_max_iters
                       << "iterations."
                       << "Transformation change:" << icp.transformationChange()
                       << "/ epsilon:" << icp.transformationEpsilon();
                     /*<< "Fitness:" << icp.euclideanFitness()
                       << "/ epsilon:" << icp.euclideanFitnessEpsilon();*/

            if(icp.hasConverged() && score < m_score_thr){
                /*debug(2) << BashColour::Green << "final transform:\n"
                         << final_transform;*/
                Eigen::Vector3f xytheta = xythetaFrom4dAffine(final_transform);
                infostream << BashColour::Green << " pairwise match:"
                           << xytheta[0] << "," << xytheta[1] << "rot=" << xytheta[2] << "deg";
                transformation = final_transform;
            }else if(score >= m_score_thr){
                infostream << BashColour::Brown
                           << " failed (error too high: "
                           << score << ">=" << m_score_thr <<")";
                throw PairwiseMatchException("error too high");
            }else{
                infostream << BashColour::Red
                           << " failed (not converged)";
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
        const int m_ransac_iters;
};


// !!! when my compiler supports template typedefs the PointT can be a template parameter
typedef _ICPPairwiseMatcher<
    pcl::PointXYZ,
    pcl::IterativeClosestPoint<
        pcl::PointXYZ, pcl::PointXYZ
    >
> ICPPairwiseMatcher;

typedef _ICPPairwiseMatcher<
    pcl::PointXYZ,
    pcl::IterativeClosestPointNonLinear<
        pcl::PointXYZ, pcl::PointXYZ
    >
> ICPNonLinearPairwiseMatcher;


// - make_shared functions:

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeICPPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float reject_threshold,
    float max_correspond_dist,
    float score_thr,
    int ransac_iters
){
    return boost::make_shared<ICPPairwiseMatcher>(
        max_iters, euclidean_fitness, transform_eps, reject_threshold, max_correspond_dist, score_thr, ransac_iters
    );
}

boost::shared_ptr<PairwiseMatcher<pcl::PointXYZ> > makeICPNonLinearPairwiseMatcherShared(
    int max_iters,
    float euclidean_fitness,
    float transform_eps,
    float reject_threshold,
    float max_correspond_dist,
    float score_thr,
    int ransac_iters
){
    return boost::make_shared<ICPNonLinearPairwiseMatcher>(
        max_iters, euclidean_fitness, transform_eps, reject_threshold, max_correspond_dist, score_thr, ransac_iters
    );
}

} // namespace cauv
} // namespace imgproc


