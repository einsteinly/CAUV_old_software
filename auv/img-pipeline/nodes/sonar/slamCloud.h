#ifndef __CAUV_SONAR_SLAM_CLOUD_H__
#define __CAUV_SONAR_SLAM_CLOUD_H__

#include <boost/enable_shared_from_this.hpp>

#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
//#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

namespace cauv{
namespace imgproc{


template<typename PointT>
class SlamCloudPart;

class PairwiseMatchException: public std::runtime_error{
    public:
};

template<typename PointT>
class PairwiseMatcher{
    public:
        // - public types    
        typedef boost::shared_ptr<SlamCloudPart<PointT> > CloudPtr;

        // - public methods
        virtual Eigen::Matrix4f transformCloudToMatch(CloudPtr map, CloudPtr new_cloud) = 0;
};

template<typename PointT>
class ICPPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        //using PairwiseMatcher<PointT>::CloudPtr;
        typedef boost::shared_ptr<SlamCloudPart<PointT> > CloudPtr;        

        // - public methods
        virtual Eigen::Matrix4f transformCloudToMatch(CloudPtr map, CloudPtr new_cloud){
            // TODO
        }
};

template<typename PointT>
class NDTPairwiseMatcher: public PairwiseMatcher<PointT>{
    public:
        // - public types
        //using PairwiseMatcher<PointT>::CloudPtr;
        typedef boost::shared_ptr<SlamCloudPart<PointT> > CloudPtr;        

        // - public methods
        virtual Eigen::Matrix4f transformCloudToMatch(CloudPtr map, CloudPtr new_cloud){
            // TODO
        }
};

template<typename PointT>
class SlamCloudPart: public pcl::PointCloud<PointT>,
                     public boost::enable_shared_from_this< SlamCloudPart<PointT> >{
    public:
        // - public types
        typedef boost::shared_ptr<SlamCloudPart<PointT> > Ptr;
        typedef boost::shared_ptr<const SlamCloudPart<PointT> > ConstPtr;
        typedef pcl::PointCloud<PointT> BaseT;

        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::size;
        using pcl::PointCloud<PointT>::push_back;

        using boost::enable_shared_from_this< SlamCloudPart<PointT> >::shared_from_this;



        
    private:
        
};

template<typename PointT>
class SlamCloudGraph{
    public:
        // - public types    
        typedef boost::shared_ptr<SlamCloudPart<PointT> > CloudPtr;
        
        // - public methods
        SlamCloudGraph()
            : Overlap_Threshold(0.5){
        }

        /* Return the relative transformation of cloud p applied in the process
         * of adding it to the map
         */
        Eigen::Matrix4f addCloud(CloudPtr p, PairwiseMatcher<PointT> const& m){
            if(!all_parts.size()){
                if(cloudIsGoodEnoughForInitialisation(p))
                    all_parts.push_back(p);
                else
                    debug() << "cloud not good enough for initialisation";
                return;
            }
                
            std::vector<CloudPtr> initial_overlaps = overlappingClouds(p);
            std::vector<CloudPtr> final_overlaps;
            Eigen::Matrix4f transformation = Eigen::Matrix4f::Zero();

            try{
                if(initial_overlaps.size() == 0){
                    // outside cloud, we're lost!
                    
                }else if(initial_overlaps.size() == 1){
                    // one pairwise match at the initial guess position
                    CloudPtr map_cloud = initial_overlaps[0];
                    transformation = m.transformCloudToMatch(map_cloud, p);
                    
                    // find new overlaps at the final position
                    final_overlaps = overlappingClouds(p);
                    
                    if(final_overlaps.size() == 0){
                        warning() << "final overlap might be very small";
                    }
                    if(final_overlaps.size() <= 1){
                        // one correspondence in existing map: if we're far
                        // enough from the previous position, add a new part
                        // to the map:
                        all_parts.push_back(p);
                    }
                }else{
                    // loop close with the initial overlaps:
                    final_overlaps = initial_overlaps;
                }
            }catch(PairwiseMatchException& e){
                error() << "failed to match cloud part with single overlap:" << e.what();
            }

            if(final_overlaps.size() > 1){
                // loop close:
                // TODO
            }
        }


    private:
        // - private methods
        /* Return all cloud parts in the map that overlap with p by more than
         * Overlap_Threshold.
         */
        std::vector<CloudPtr> overlappingClouds(CloudPtr p) const{
        }
        
        /* Judge degree of overlap: new parts are added to the map when the
         * overlap with existing parts is less than Overlap_Threshold
         */
        float overlapPercent(CloudPtr a, CloudPtr b) const{
        }

        /* judge how good initial cloud is by a combination of the number of
         * features, and how well it matches with itself, or something.
         */
        bool cloudIsGoodEnoughForInitialisation(CloudPtr p) const{
        }
        
        // - private data
        const float Overlap_Threshold;

        // TODO: to remain efficient there MUST be a way of searching for
        // SlamCloudParts near a location without iterating through all nodes
        // (kdtree probably)
        std::vector<CloudPtr> all_parts;
};








// deprecated:

typedef float descriptor_t;


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
              m_transformation(Eigen::Matrix4f::Identity()){
        }
        
        Eigen::Matrix4f& transformation(){ return m_transformation; }
        Eigen::Matrix4f const& transformation() const{ return m_transformation; }

        std::vector<descriptor_t>& descriptors(){ return m_point_descriptors; }
        std::vector<descriptor_t> const& descriptors() const{ return m_point_descriptors; }

        /* Merge two clouds, merging each point from the source cloud with it's
         * nearest neighbour in the target cloud if that neighbour is closer
         * than merge_distance.
         */
        void mergeCollapseNearest(Ptr source, float const& merge_distance, std::vector<int>& keypoint_goodness){
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
                keypoint_goodness[c.first] |= 1;
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
                    keypoint_goodness[i] = 0;
                output->push_back(source->points[i], source->m_point_descriptors[i]);
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

        inline void push_back(PointT const& p, descriptor_t const& d){
            BaseT::push_back(p);
            m_point_descriptors.push_back(d);
        }

    private:
        // - private data

        // point descriptors
        std::vector<descriptor_t> m_point_descriptors;

        // transformation that has been applied to this cloud
        Eigen::Matrix4f m_transformation;
};

} // namespace cauv
} // namespace imgproc

#endif //ndef __CAUV_SONAR_SLAM_CLOUD_H__
