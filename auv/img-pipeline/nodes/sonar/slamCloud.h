#ifndef __CAUV_SONAR_SLAM_CLOUD_H__
#define __CAUV_SONAR_SLAM_CLOUD_H__

#include <deque>

#include <boost/enable_shared_from_this.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

namespace cauv{
namespace imgproc{

// - Typedefs
typedef float descriptor_t;

// - Forward Declarations
class SlamCloudLocation;
template<typename PointT> class SlamCloudPart;
template<typename PointT> class SlamCloudGraph;

// - Static Utility Functions
static Eigen::Vector3f xythetaFrom4dAffine(Eigen::Matrix4f const& transform){
    // split transform into affine parts:
    const Eigen::Matrix3f rotate    = transform.block<3,3>(0, 0);
    const Eigen::Vector3f translate = transform.block<3,1>(0, 3);

    const Eigen::Vector3f t = rotate*Eigen::Vector3f(1,0,0);
    const float rz = (180/M_PI)*std::atan2(t[1], t[0]);
    return Eigen::Vector3f(translate[0], translate[1], rz);
}

// return value in double-precision floating point seconds (musec precision)
static double operator-(cauv::TimeStamp const& left, cauv::TimeStamp const& right){
    const double dsecs = left.secs - right.secs;
    const double dmusecs = left.musecs - right.musecs;
    return dsecs + 1e6*dmusecs;
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
        typedef boost::shared_ptr<SlamCloudPart<PointT> > cloud_ptr;

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
            cloud_ptr map,
            cloud_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation
        ) const = 0;
};

template<typename PointT>
class ICPPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<SlamCloudPart<PointT> > cloud_ptr;

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
            cloud_ptr map,
            cloud_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation
        ) const {
            debug() << "ICPPairwiseMatcher" << map->size() << ":" << new_cloud->size() << "points";

            //pcl::IterativeClosestPointNonLinear<PointT,PointT> icp;
            pcl::IterativeClosestPoint<PointT,PointT> icp;
            icp.setInputCloud(new_cloud);
            icp.setInputTarget(map);

            icp.setMaxCorrespondenceDistance(m_max_correspond_dist);
            icp.setMaximumIterations(m_max_iters);
            icp.setTransformationEpsilon(m_transform_eps);
            icp.setEuclideanFitnessEpsilon(m_euclidean_fitness);
            icp.setRANSACOutlierRejectionThreshold(m_reject_threshold);

            const Eigen::Matrix4f relative_guess = map->relativeTransform().inverse() * guess * new_cloud->globalTransform();

            debug() << BashColour::Green << "guess:\n"
                    << guess;
            debug() << BashColour::Green << "relative guess:\n"
                    << relative_guess;

            // do the hard work!
            typename cloud_t::base_cloud_t::Ptr final = boost::make_shared<typename cloud_t::base_cloud_t>();
            icp.align(*final, relative_guess);
            // in map's coordinate system:
            const Eigen::Matrix4f final_transform = icp.getFinalTransformation();

            // high is bad (score is sum of squared euclidean distances)
            const float score = icp.getFitnessScore();
            info() << BashColour::Green
                   << "converged:" << icp.hasConverged()
                   << "score:" << score;

            if(icp.hasConverged() && score < m_score_thr){
                debug() << BashColour::Green << "final transform\n:"
                        << final_transform;
                Eigen::Vector3f xytheta = xythetaFrom4dAffine(final_transform);
                info() << BashColour::Green << "pairwise match:"
                       << xytheta[0] << "," << xytheta[1] << "rot=" << xytheta[2] << "deg";

                transformation = final_transform;
            }else if(score >= m_score_thr){
                info() << BashColour::Brown
                       << "points will not be added to the whole cloud (error too high: "
                       << score << ">=" << m_score_thr <<")";
                throw PairwiseMatchException("error too high");
            }else{
                info() << BashColour::Red
                       << "points will not be added to the whole cloud (not converged)";
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
        //using PairwiseMatcher<PointT>::cloud_ptr;
        typedef boost::shared_ptr<SlamCloudPart<PointT> > cloud_ptr;

        // - public methods
        virtual float transformcloudToMatch(
            cloud_ptr map,
            cloud_ptr new_cloud,
            Eigen::Matrix4f const& guess,
            Eigen::Matrix4f& transformation
        ) const {
            // TODO
        }
};

// - SLAM Clouds
class SlamCloudLocation{
    public:
        typedef boost::shared_ptr<SlamCloudLocation> location_ptr;

        SlamCloudLocation(TimeStamp const& t)
            : m_relative_to(),
              m_relative_transformation(Eigen::Matrix4f::Identity()),
              m_time(t){
        }

        virtual ~SlamCloudLocation(){
        }

        template<typename PointT>
        explicit SlamCloudLocation(boost::shared_ptr<SlamCloudPart<PointT> > const& p)
            : m_relative_to(p->m_relative_to),
              m_relative_transformation(p->m_relative_transformation),
              m_time(p->m_time){
        }

        // post-multiply
        void transform(Eigen::Matrix4f const& transformation){
            m_relative_transformation = m_relative_transformation * transformation;
        }
        
        void setRelativeToNone(){
            m_relative_to.reset();
        }
        void setRelativeTo(location_ptr p){
            m_relative_to = p;
        }

        void setRelativeTransform(Eigen::Matrix4f const& m){
            m_relative_transformation = m;
        }

        Eigen::Matrix4f globalTransform() const{
            Eigen::Matrix4f r = relativeTransform();
            location_ptr p = m_relative_to;
            while(p){
                r = p->relativeTransform() * r;
                p = p->relativeTo();
            }
            return r;
        }

        Eigen::Matrix4f const& relativeTransform() const{ return m_relative_transformation; }
        location_ptr relativeTo() const{ return m_relative_to; }
        TimeStamp const& time() const{ return m_time; }

    private:
        location_ptr    m_relative_to;              // TODO: list?
        Eigen::Matrix4f m_relative_transformation;  // TODO: list?
        TimeStamp       m_time;
};

template<typename PointT>
class SlamCloudPart: public SlamCloudLocation,
                     public pcl::PointCloud<PointT>,
                     public boost::enable_shared_from_this< SlamCloudPart<PointT> >{
    public:
        // - public types
        typedef pcl::PointCloud<PointT> base_cloud_t;
        typedef boost::shared_ptr<SlamCloudPart<PointT> > Ptr;
        typedef boost::shared_ptr<const SlamCloudPart<PointT> > ConstPtr;
        typedef pcl::PointCloud<PointT> BaseT;

        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::size;
        using pcl::PointCloud<PointT>::push_back;

        using boost::enable_shared_from_this< SlamCloudPart<PointT> >::shared_from_this;

        struct FilterResponse{
            FilterResponse(float thr)
                : m_thr(thr){
            }
            bool operator()(KeyPoint const& k) const{
                return k.response > m_thr;
            }
            const float m_thr;
        };
        template<typename Callable>
        SlamCloudPart(std::vector<KeyPoint> const& kps, TimeStamp const& t, Callable const& filter)
            : SlamCloudLocation(t),
              pcl::PointCloud<PointT>(),
              boost::enable_shared_from_this< SlamCloudPart<PointT> >(),
              m_point_descriptors(),
              m_keypoint_indices(),
              m_keypoint_goodness(kps.size(), 0){

            this->height = 1;
            this->is_dense = true;
            reserve(kps.size());

            for(size_t i = 0; i < kps.size(); i++){
                if(!filter(kps[i]))
                    continue;
                push_back(PointT(kps[i].pt.x, kps[i].pt.y, 0.0f), kps[i].response, i);
                // start out assuming all keypoints that have passed the weight
                // test are good:
                m_keypoint_goodness[i] = 1;
            }
        }

        std::vector<descriptor_t>& descriptors(){ return m_point_descriptors; }
        std::vector<descriptor_t> const& descriptors() const{ return m_point_descriptors; }

        std::vector<std::size_t>& ptIndices(){ return m_keypoint_indices; }
        std::vector<std::size_t> const& ptIndices() const{ return m_keypoint_indices; }

        std::vector<int>& keyPointGoodness(){ return m_keypoint_goodness; }
        std::vector<int> const& keyPointGoodness() const{ return m_keypoint_goodness; }

        // "overridden" methods: note that none of the base class methods are
        // virtual, so these are only called if the call is made through a
        // pointer to this derived type
        void reserve(std::size_t s){
            BaseT::reserve(s);
            m_point_descriptors.reserve(s);
        }

        inline void push_back(PointT const& p, descriptor_t const& d, std::size_t const& idx){
            BaseT::push_back(p);
            m_point_descriptors.push_back(d);
            m_keypoint_indices.push_back(idx);
        }

    private:
        std::vector<descriptor_t> m_point_descriptors;

        // original indices of the keypoints from which the points in this
        // cloud were derived - used to generate training data for those
        // keypoints: these are not generally preserved by operations on the
        // point cloud
        std::vector<std::size_t> m_keypoint_indices;

        // 1 = keypoint turned out to be good, 0 = keypoint turned out to be
        // bad
        std::vector<int> m_keypoint_goodness;
};

template<typename PointT>
class SlamCloudGraph{
    public:
        // - public types
        typedef boost::shared_ptr<SlamCloudPart<PointT> > cloud_ptr;
        typedef boost::shared_ptr<SlamCloudLocation> location_ptr;

    private:
        // - private types
        typedef std::deque<location_ptr> location_vec;
        typedef std::deque<cloud_ptr> cloud_vec;

    public:
        // - public methods
        SlamCloudGraph()
            : Overlap_Threshold(0.3),
              Keyframe_Spacing(10),
              Min_Initial_Points(10){
        }

        void reset(){
            key_scans.clear();
            all_scans.clear();
        }

        /* Guess a transformation for a time based on simple extrapolation of
         * the previous transformations.
         * Returned transformation is in the global coordinate system
         */
        Eigen::Matrix4f guessTransformationAtTime(TimeStamp const& t) const{
            switch(all_scans.size()){
                case 0:
                    return Eigen::Matrix4f::Identity();
                case 1:
                    assert(!all_scans.back()->relativeTo());
                    return all_scans.back()->globalTransform();
                default:{
                    location_vec::const_reverse_iterator i = all_scans.rbegin();
                    const Eigen::Matrix4f r1 = (*i)->globalTransform();
                    const TimeStamp t1 = (*i)->time();
                    const Eigen::Matrix4f r2 = (*++i)->globalTransform();
                    const TimeStamp t2 = (*i)->time();
                    return r2 + (r2-r1)*(t - t2)/(t2 - t1);
                }
            }
        }

        /* Match scan 'p' to the map, return the confidence in the match.
         * 'transformation' is set to the global transformation of the new
         * scan.
         */
        float registerScan(cloud_ptr p,
                           Eigen::Matrix4f const& guess,
                           PairwiseMatcher<PointT> const& m,
                           Eigen::Matrix4f& transformation){
            if(!key_scans.size()){
                if(cloudIsGoodEnoughForInitialisation(p)){
                    key_scans.push_back(p);
                    transformation = Eigen::Matrix4f::Identity();
                    return 1.0f;
                }else{
                    debug() << "cloud not good enough for initialisation";
                    return 0.0f;
                }
            }

            const cloud_vec initial_overlaps = overlappingClouds(p, guess);
            cloud_vec final_overlaps;
            Eigen::Matrix4f relative_transformation;
            float r = 0.0f;

            try{
                if(initial_overlaps.size() == 0){
                    debug() << "new scan falls outside map!";
                    return 0;
                }else if(initial_overlaps.size() == 1){
                    // one pairwise match at the initial guess position
                    cloud_ptr map_cloud = initial_overlaps[0];
                    r = m.transformcloudToMatch(map_cloud, p, guess, relative_transformation);
                    p->setRelativeTransform(relative_transformation);
                    p->setRelativeTo(map_cloud);
                    // we apply transformations by pre-multiplying, so
                    // post-multiply the transformation that should be applied
                    // first.
                    transformation = relative_transformation * map_cloud->relativeTransform();

                    // find new overlaps at the final position
                    final_overlaps = overlappingClouds(p);

                    if(final_overlaps.size() == 0){
                        warning() << "final overlap too small";
                        return 0;
                    }
                    if(final_overlaps.size() == 1){
                        if(final_overlaps[0] != initial_overlaps[0]){
                            warning() << "final overlap different";
                            return 0;
                        }
                        // one correspondence in existing map: if we're far
                        // enough from the previous position, add a new part
                        // to the map:
                        Eigen::Vector3f relative_displacement = relative_transformation.block<3,1>(0, 3);
                        if(relative_displacement.norm() > Keyframe_Spacing){
                            // key scans are transformed to absolute coordinate
                            // frame?
                            // !!! TODO: should they be relative to each other?
                            p->setRelativeTransform(p->globalTransform());
                            p->setRelativeToNone();
                            key_scans.push_back(p);
                            all_scans.push_back(p);
                            debug() << "key frame at" << transformation.block<3,1>(0, 3);
                        }else{
                            // discard all the point data for non-key scans
                            all_scans.push_back(boost::make_shared<SlamCloudLocation>(p));
                            debug() << "non-key frame at" << transformation.block<3,1>(0, 3);
                        }
                    }
                }else{
                    debug() << "multiple significant initial overlaps";
                    // loop close with the initial overlaps:
                    final_overlaps = initial_overlaps;
                }
            }catch(PairwiseMatchException& e){
                error() << "failed to match cloud part with single overlap:" << e.what();
                return 0;
            }

            if(final_overlaps.size() > 1){
                warning() << "loop closing not implemented!";
                // loop close:
                // TODO
            }

            return r;
        }

    private:
        // - private methods
        /* Return all cloud parts in the map that overlap with p by more than
         * Overlap_Threshold.
         *
         */
        cloud_vec overlappingClouds(cloud_ptr p) const{

        }

        /* overload to find overlaps at a different transformation to the
         * current one (additional transformation is applied last, ie:
         * global_point = p->relativeTo().relativeTransform() *  p->relativeTransform() * additional_transform * cloud_point;
         */
        cloud_vec overlappingClouds(cloud_ptr p, Eigen::Matrix4f const& additional_transform){
            Eigen::Matrix4f saved_transform = p->relativeTransform();
            p->setRelativeTransform(saved_transform * additional_transform);
            cloud_vec r = overlappingClouds(p);
            p->setRelativeTransform(saved_transform);
            return r;
        }

        /* Judge degree of overlap: new parts are added to the map when the
         * overlap with existing parts is less than Overlap_Threshold
         */
        float overlapPercent(cloud_ptr a, cloud_ptr b) const{
        }

        /* judge how good initial cloud is by a combination of the number of
         * features, and how well it matches with itself, or something.
         */
        bool cloudIsGoodEnoughForInitialisation(cloud_ptr p) const{
            // !!! TODO: more sophisticated check
            return (p->size() > Min_Initial_Points);
        }

        // - private data
        const float Overlap_Threshold; // a fraction (0--1)
        const float Keyframe_Spacing;  // in metres
        const float Min_Initial_Points; // for first keyframe

        // TODO: to remain efficient there MUST be a way of searching for
        // SlamCloudParts near a location without iterating through all nodes
        // (kdtree probably)
        cloud_vec key_scans;

        // for these scans, all data apart from the time and relative location
        // is discarded
        location_vec all_scans;
};
























// - deprecated


template<typename PointT>
class SlamCloud: public pcl::PointCloud<PointT>,
                 public boost::enable_shared_from_this< SlamCloud<PointT> >{
    public:
        // - public types
        typedef boost::shared_ptr<SlamCloud<PointT> > Ptr;
        typedef boost::shared_ptr<const SlamCloud<PointT> > ConstPtr;
        typedef pcl::PointCloud<PointT> BaseT;

        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::size;
        using pcl::PointCloud<PointT>::push_back;

        using boost::enable_shared_from_this< SlamCloud<PointT> >::shared_from_this;

    public:
        // - public methods
        SlamCloud()
            : BaseT(),
              m_point_descriptors(),
              m_keypoint_indices(),
              m_transformation(Eigen::Matrix4f::Identity()){
        }

        Eigen::Matrix4f& transformation(){ return m_transformation; }
        Eigen::Matrix4f const& transformation() const{ return m_transformation; }

        std::vector<descriptor_t>& descriptors(){ return m_point_descriptors; }
        std::vector<descriptor_t> const& descriptors() const{ return m_point_descriptors; }

        std::vector<std::size_t>& ptIndices(){ return m_keypoint_indices; }
        std::vector<std::size_t> const& ptIndices() const{ return m_keypoint_indices; }


        /* Merge two clouds, merging each point from the source cloud with it's
         * nearest neighbour in the target cloud if that neighbour is closer
         * than merge_distance.
         */
        void mergeCollapseNearest(Ptr source, float const& merge_distance, std::vector<int>& keypoint_goodness){
            assert(keypoint_goodness.size() >= source->m_keypoint_indices.size());

            pcl::KdTreeFLANN<PointT> kdtree;
            kdtree.setInputCloud(shared_from_this());
            typedef std::pair<int,int> idx_pair;
            std::vector<idx_pair> correspondences;
            std::vector<int> no_correspondence;
            std::vector<int>   pt_indices(1);
            std::vector<float> pt_squared_dists(1);

            for(size_t i=0; i < source->size(); i++){
                if(kdtree.nearestKSearch((*source)[i], 1, pt_indices, pt_squared_dists) > 0 &&
                   pt_squared_dists[0] < merge_distance){
                    correspondences.push_back(std::pair<int,int>(i, pt_indices[0]));
                }else{
                    no_correspondence.push_back(i);
                }
            }
            debug() << "merge:" << correspondences.size() << "correspondences /" << source->size() << "points";

            foreach(idx_pair const& c, correspondences){
                // equal weighting of old a new points... might want to give
                // stronger weight to old points
                points[c.second].getVector3fMap() = (points[c.second].getVector3fMap() + (*source)[c.first].getVector3fMap()) / 2;

                // current descriptors are scalar, and just sum!
                m_point_descriptors[c.second] += source->descriptors()[c.first];

                // this was a good keypoint in the input cloud
                keypoint_goodness[source->m_keypoint_indices[c.first]] |= 1;
            }

            if(no_correspondence.size()){
                reserve(size()+no_correspondence.size());
                m_point_descriptors.reserve(size()+no_correspondence.size());
                foreach(int i, no_correspondence){
                    push_back((*source)[i]);
                    m_point_descriptors.push_back(source->descriptors()[i]);
                }
                width = size();
            }
        }

        // keypoint_goodness should start out set to 1, elements which weren't
        // good will be set to 0
        void mergeOutsideConcaveHull(Ptr source, float const& alpha/*= 5.0f*/,
                                     float const& merge_distance/*=0.0f*/,
                                     std::vector<int>& keypoint_goodness){
            pcl::ConcaveHull<PointT> hull_calculator;
            typename BaseT::Ptr hull(new BaseT);
            std::vector<pcl::Vertices> polygons;

            assert(keypoint_goodness.size() >= source->m_keypoint_indices.size());

            hull_calculator.setInputCloud(shared_from_this());
            hull_calculator.setAlpha(alpha);
            hull_calculator.reconstruct(*hull, polygons);

            int dim = hull_calculator.getDim();
            if(dim != 2)
                throw std::runtime_error("3D hull!");

            debug() << "hull has" << hull->size() << "points:";

            pcl::CropHull<PointT> crop_filter;
            crop_filter.setInputCloud(source);
            crop_filter.setHullCloud(hull);
            crop_filter.setHullIndices(polygons);
            crop_filter.setDim(dim);
            crop_filter.setCropOutside(false);

            std::vector<int> output_indices;
            crop_filter.filter(output_indices);
            debug() << output_indices.size() << "/" << source->size() << "passed filter";

            Ptr output(new SlamCloud<PointT>);
            output->m_transformation = source->m_transformation;
            output->reserve(output_indices.size());
            foreach(int i, output_indices){
                // for the points that passed the crop test, default to being
                // bad, unless they are merged with an existing point:
                if(merge_distance != 0.0f)
                    keypoint_goodness[source->m_keypoint_indices[i]] = 0;
                output->push_back(source->points[i], source->m_point_descriptors[i], source->m_keypoint_indices[i]);
            }

            if(merge_distance == 0.0f){
                BaseT::operator+=(*output);
                m_point_descriptors.insert(
                    m_point_descriptors.end(),
                    output->m_point_descriptors.begin(),
                    output->m_point_descriptors.end()
                );
            }else{
                mergeCollapseNearest(output, merge_distance, keypoint_goodness);
            }
        }

        // "overridden" methods: note that none of the base class methods are
        // virtual, so these are only called if the call is made through a
        // pointer to this derived type
        void reserve(std::size_t s){
            BaseT::reserve(s);
            m_point_descriptors.reserve(s);
        }

        inline void push_back(PointT const& p, descriptor_t const& d, std::size_t const& idx){
            BaseT::push_back(p);
            m_point_descriptors.push_back(d);
            m_keypoint_indices.push_back(idx);
        }

    private:
        // - private data

        // point descriptors
        std::vector<descriptor_t> m_point_descriptors;

        // original indices of the keypoints from which the points in this
        // cloud were derived - used to generate training data for those
        // keypoints: these are not generally preserved by operations on the
        // point cloud
        std::vector<std::size_t> m_keypoint_indices;

        // transformation that has been applied to this cloud
        Eigen::Matrix4f m_transformation;
};

} // namespace cauv
} // namespace imgproc

#endif //ndef __CAUV_SONAR_SLAM_CLOUD_H__
