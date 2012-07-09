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

#ifndef __CAUV_SONAR_SLAM_CLOUD_PART_H__
#define __CAUV_SONAR_SLAM_CLOUD_PART_H__

#include <vector>
#include <fstream>

#include <boost/enable_shared_from_this.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/convex_hull.h>

#include <Eigen/StdVector>

#include <debug/cauv_debug.h>

#include <generated/types/TimeStamp.h>
#include <generated/types/KeyPoint.h>

#include <utility/foreach.h>

#include "graphOptimiser.h"
#include "common.h"

namespace cauv{
namespace imgproc{


// - SLAM Clouds
class SlamCloudLocation{
    public:
        typedef std::vector< RelativePose, Eigen::aligned_allocator<RelativePose> > rel_pose_vec;

        SlamCloudLocation(TimeStamp const& t)
            : m_relative_to(),
              m_relative_transformation(Eigen::Matrix4f::Identity()),
              m_time(t){
            _checkAlignment();
        }

        template<typename PointT>
        explicit SlamCloudLocation(boost::shared_ptr<SlamCloudPart<PointT> > const& p)
            : m_relative_to(p->m_relative_to),
              m_relative_transformation(p->m_relative_transformation),
              m_time(p->m_time){
            _checkAlignment();
        }

        explicit SlamCloudLocation(float x, float y, float theta_radians)
            : m_relative_to(), 
              m_relative_transformation(Eigen::Matrix4f::Identity()),
              m_time(){
            m_relative_transformation.block<2,1>(0,3) = Eigen::Vector2f(x, y);
            m_relative_transformation.block<2,2>(0,0) *= Eigen::Matrix2f(Eigen::Rotation2D<float>(theta_radians));
            _checkAlignment();
        }

        virtual ~SlamCloudLocation(){
        }

        // post-multiply
        void transform(Eigen::Matrix4f const& transformation){
            m_relative_transformation = m_relative_transformation * transformation;
            transformationChanged();
        }

        void setRelativeToNone(){
            if(m_relative_to){
                m_relative_to.reset();
                transformationChanged();
            }
        }
        void setRelativeTo(location_ptr p){
            if(p != m_relative_to){
                m_relative_to = p;
                transformationChanged();
            }
        }
        
        // add constraint from this -> p (p observed from this) t is the
        // RELATIVE (not incremental) pose at which p is observed
        RelativePose addConstraintTo(location_ptr p, Eigen::Matrix4f const& t){
            const RelativePose r = RelativePose::from4dAffine(t);
            
            m_constrained_to.push_back(p);
            m_constraints.push_back(r);
            
            return r;
        }

        static pose_constraint_ptr addConstraintBetween(location_ptr from,
                                                        location_ptr to,
                                                        Eigen::Matrix4f from_to_to){
            const RelativePose rel_pose = from->addConstraintTo(to, from_to_to);
            return boost::make_shared<RelativePoseConstraint>(rel_pose, from, to);
        }

        void setRelativeTransform(Eigen::Matrix4f const& m){
            m_relative_transformation = m;
            transformationChanged();            
        }

        Eigen::Matrix4f globalTransform() const{
            Eigen::Matrix4f r = relativeTransform();
            location_ptr p = relativeTo();
            if(p)
                return p->globalTransform() * r;
            return r;
        }

        Eigen::Matrix4f const& relativeTransform() const{ return m_relative_transformation; }
        location_ptr relativeTo() const{ return m_relative_to; }
        TimeStamp const& time() const{ return m_time; }
        
        // !!! constrainedTo should be in constraints
        location_vec const& constrainedTo() const{ return m_constrained_to; }
        rel_pose_vec const& constraints() const{ return m_constraints; }

        // (relatively) expensive: use sparingly
        typedef std::vector<Eigen::Vector3f,Eigen::aligned_allocator<Eigen::Vector3f> > v3f_vec;
        v3f_vec constraintEndsGlobal() const{
            v3f_vec r;
            foreach(RelativePose const& rel, m_constraints){
                const Eigen::Matrix4f pt_transform = globalTransform() * rel.to4dAffine();
                const Eigen::Vector3f pt = pt_transform.block<3,1>(0,3);
                r.push_back(pt);
            }
            return r;
        }
        

        // We have an Eigen::Matrix4f as a member
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        virtual void transformationChanged(){}

    private:
        void _checkAlignment() const{
            #ifndef CAUV_NO_DEBUG
            if(int64_t(&m_relative_transformation) & 0x7)
                error() << "Bad Alignment of SlamCloudLocation: this:" << this
                        << "m_rel_tr:" << &m_relative_transformation
                        << "Bad things will happen!";
            #endif
        }

    
        location_ptr    m_relative_to;
        Eigen::Matrix4f m_relative_transformation;

        location_vec  m_constrained_to;
        rel_pose_vec  m_constraints;

        TimeStamp       m_time;
};

template<typename PointT>
class KDTreeCachingCloud: public pcl::PointCloud<PointT>,
                          public boost::enable_shared_from_this< KDTreeCachingCloud<PointT> >{
    public:
        // - public types
        typedef boost::shared_ptr<SlamCloudPart<PointT> > Ptr;
        typedef boost::shared_ptr<const SlamCloudPart<PointT> > ConstPtr;
        typedef pcl::PointCloud<PointT> base_cloud_t;
        typedef typename base_cloud_t::Ptr base_cloud_ptr;

        using boost::enable_shared_from_this< KDTreeCachingCloud<PointT> >::shared_from_this;
    
        KDTreeCachingCloud()
            : pcl::PointCloud<PointT>(), 
              boost::enable_shared_from_this< KDTreeCachingCloud<PointT> >(),
              m_kdtree(),
              m_kdtree_invalid(true){
            if(int64_t((void*)dynamic_cast<pcl::PointCloud<PointT>*>(this)) & 0x7)
                error() << "Bad alignment of KDTreeCachingCloud this:" << this
                        << "PointCloud<PointT> base:" << dynamic_cast<pcl::PointCloud<PointT>*>(this)
                        << "Bad things will happen!";
        }

        virtual ~KDTreeCachingCloud(){
        }
        
        inline void push_back(PointT const& p){
            base_cloud_t::push_back(p);
            m_kdtree_invalid = true;
        }

        inline void clear(){
            base_cloud_t::clear();
            m_kdtree_invalid = true;
        }

        int nearestKSearch(PointT const& point,
                            int k,
                            std::vector<int> &k_indices,
                            std::vector<float> &k_sqr_distances){
            if(!base_cloud_t::size())
                return 0;
            ensureKdTree();
            return m_kdtree.nearestKSearch(point, k, k_indices, k_sqr_distances);
        }

        float nearestSquaredDist(PointT const& point){
            if(!base_cloud_t::size())
                return std::numeric_limits<float>::max();
            std::vector<int> k_indices(1);
            std::vector<float> k_sqr_distances(1);
            ensureKdTree();
            m_kdtree.nearestKSearch(point, 1, k_indices, k_sqr_distances);
            return k_sqr_distances[0];
        }
    
        void invalidateKDTree(){
            m_kdtree_invalid = true;
        }
        
        // derives from stuff with Eigen::stuff as members
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
    private:
        void ensureKdTree(){
            if(m_kdtree_invalid){
                m_kdtree_invalid = false;
                m_kdtree = pcl::KdTreeFLANN<PointT>();
                m_kdtree.setInputCloud(shared_from_this());
                // clear the circular reference just created through call to
                // base version of setInputCloud, which doesn't clobber the
                // kdtree...
                m_kdtree.pcl::template KdTree<PointT>::setInputCloud(base_cloud_ptr());
           }
        }

        pcl::KdTreeFLANN<PointT> m_kdtree;
        bool m_kdtree_invalid;
};

template<typename PointT>
class SlamCloudPart: public SlamCloudLocation,
                     public KDTreeCachingCloud<PointT>{
    public:
        // - public types
        typedef boost::shared_ptr<SlamCloudPart<PointT> > Ptr;
        typedef boost::shared_ptr<const SlamCloudPart<PointT> > ConstPtr;
        typedef pcl::PointCloud<PointT> base_cloud_t;
        typedef typename base_cloud_t::Ptr base_cloud_ptr;

        using pcl::PointCloud<PointT>::points;
        using pcl::PointCloud<PointT>::width;
        using pcl::PointCloud<PointT>::size;
        using pcl::PointCloud<PointT>::push_back;

        using KDTreeCachingCloud<PointT>::shared_from_this;

        struct FilterResponse{
            FilterResponse(float thr)
                : m_thr(thr){
            }
            bool operator()(KeyPoint const& k) const{
                return k.response > m_thr;
            }
            const float m_thr;
        };
            
        enum {Keyframe_Serialise_Version = 1};

        // NB: this function converts from OpenCV image-space keypoints (y-axis
        // downwards) to a y-axis upwards convention
        template<typename Callable>
        SlamCloudPart(std::vector<KeyPoint> const& kps, TimeStamp const& t, Callable const& filter)
            : SlamCloudLocation(t),
              KDTreeCachingCloud<PointT>(),
              m_point_descriptors(),
              m_keypoint_indices(),
              m_rejected_keypoint_indices(),
              m_keypoint_goodness(kps.size(), 0),
              m_local_convexhull_verts(),
              m_local_convexhull_cloud(),
              m_local_convexhull_invalid(true),
              m_global_convexhull_cloud(),
              m_global_convexhull_invalid(true),
              m_rejected_points_cloud(),
              m_mean(Eigen::Vector3f::Zero()),
              m_covar(Eigen::Matrix3f::Zero()){

            this->height = 1;
            this->is_dense = true;
            reserve(kps.size());

            for(size_t i = 0; i < kps.size(); i++){
                if(!filter(kps[i])){
                    m_rejected_points_cloud.push_back(PointT(kps[i].pt.x, -kps[i].pt.y, 0.0f));
                    m_rejected_keypoint_indices.push_back(i);
                    continue;
                }
                // flip the y-axis: opencv y axis is downwards
                push_back(PointT(kps[i].pt.x, -kps[i].pt.y, 0.0f), kps[i].response, i);
                // start out assuming all keypoints that have passed the weight
                // test are good:
                m_keypoint_goodness[i] = 1;
            }
        }

        virtual ~SlamCloudPart(){
        }

        void getLocalConvexHull(base_cloud_ptr& hull_points,
                                std::vector<pcl::Vertices>& polygons){
            ensureLocalConvexHull();
            hull_points = m_local_convexhull_cloud;
            polygons = m_local_convexhull_verts;
        }

        void getGlobalConvexHull(base_cloud_ptr& hull_points,
                                std::vector<pcl::Vertices>& polygons){
            ensureGlobalConvexHull();
            hull_points = m_global_convexhull_cloud;
            polygons = m_local_convexhull_verts; // NB: local
        }

        Eigen::Vector3f mean() const{
            ensureMeanVar();
            return m_mean;
        }

        Eigen::Matrix3f covar() const{
            ensureMeanVar();
            return m_covar;
        }

        std::vector<descriptor_t>& descriptors(){ return m_point_descriptors; }
        std::vector<descriptor_t> const& descriptors() const{ return m_point_descriptors; }

        std::vector<std::size_t>& ptIndices(){ return m_keypoint_indices; }
        std::vector<std::size_t> const& ptIndices() const{ return m_keypoint_indices; }

        std::vector<std::size_t>& rejectedPtIndices(){ return m_rejected_keypoint_indices; }
        std::vector<std::size_t> const& rejectedPtIndices() const{ return m_rejected_keypoint_indices; }

        std::vector<int>& keyPointGoodness(){ return m_keypoint_goodness; }
        std::vector<int> const& keyPointGoodness() const{ return m_keypoint_goodness; }

        // "overridden" methods: note that none of the base class methods are
        // virtual, so these are only called if the call is made through a
        // pointer to this derived type
        void reserve(std::size_t s){
            base_cloud_t::reserve(s);
            m_point_descriptors.reserve(s);
        }

        inline void push_back(PointT const& p, descriptor_t const& d, std::size_t const& idx){
            KDTreeCachingCloud<PointT>::push_back(p);
            m_point_descriptors.push_back(d);
            m_keypoint_indices.push_back(idx);
            m_local_convexhull_invalid = true;
            m_global_convexhull_invalid = true;
        }
        
        base_cloud_t const& rejectedPoints() const{
            return m_rejected_points_cloud;
        }

        void saveToFile(std::ofstream& f){
            uint32_t x = Keyframe_Serialise_Version;        
            f.write((char*)&x, sizeof(x));
            const TimeStamp t = time();
            f.write((char*)&t, sizeof(t));
            const std::size_t s = size();
            f.write((char*)&s, sizeof(s));
            for(std::size_t i = 0; i < s; i++){
                const float x = KDTreeCachingCloud<PointT>::operator[](i).x;
                const float y = KDTreeCachingCloud<PointT>::operator[](i).y;
                const float response = m_point_descriptors[i];
                // don't need to save the indices
                //const std::size_t idx = m_keypoint_indices[i];
                f.write((char*)&x, sizeof(x));
                f.write((char*)&y, sizeof(y));
                f.write((char*)&response, sizeof(response));
                //f.write((char*)&idx, sizeof(idx));
            }
        }

        static SlamCloudPart<PointT> loadFromFile(std::ifstream& f){
            uint32_t x = 0;
            f.read((char*)&x, sizeof(x));
            if(x != Keyframe_Serialise_Version)
                throw std::runtime_error("unknown map version");
            TimeStamp t;
            f.read((char*)&t, sizeof(t));
            SlamCloudPart r(t);
            std::size_t s = 0;
            f.read(s, sizeof(s));
            for(std::size_t i = 0; i < s; i++){
                float x = 0;
                float y = 0;
                float z = 0;
                float response = 0;
                std::size_t idx = 0;
                f.read((char*)&x, sizeof(x));
                f.read((char*)&y, sizeof(y));
                // no z
                f.read((char*)&response, sizeof(response));
                //f.read((char*)&idx, sizeof(idx));
                r.push_back(PointT(x, y, z), response, i);
            }

        }

        // this type derives from something with an Eigen::Matrix4f as a
        // member:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    protected:
        virtual void transformationChanged(){
            this->invalidateKDTree();
            m_global_convexhull_invalid = true;
        }

        
        void invalidateMeanVarCache(){
            m_meanvar_invalid = true;
        }


        SlamCloudPart(TimeStamp const& t)
            : SlamCloudLocation(t),
              KDTreeCachingCloud<PointT>(),
              m_point_descriptors(),
              m_keypoint_indices(),
              m_rejected_keypoint_indices(),
              m_keypoint_goodness(),
              m_local_convexhull_verts(),
              m_local_convexhull_cloud(),
              m_local_convexhull_invalid(true),
              m_global_convexhull_cloud(),
              m_global_convexhull_invalid(true),
              m_rejected_points_cloud(),
              m_mean(Eigen::Vector3f::Zero()),
              m_covar(Eigen::Matrix3f::Zero()){
        }

    private:
        void ensureLocalConvexHull(){
            if(m_local_convexhull_invalid){
                m_local_convexhull_invalid = false;

                pcl::ConvexHull<PointT> hull_calculator;
                #if PCL_VERSION >= PCL_VERSION_CALC(1,5,0)
                hull_calculator.setDimension(2);
                #endif
                m_local_convexhull_cloud = (boost::make_shared<base_cloud_t>());
                m_local_convexhull_cloud->is_dense = true;

                hull_calculator.setInputCloud(shared_from_this());
                hull_calculator.reconstruct(*m_local_convexhull_cloud, m_local_convexhull_verts);
            }
        }

        void ensureGlobalConvexHull(){
            if(m_global_convexhull_invalid){
                m_global_convexhull_invalid = false;
                ensureLocalConvexHull();
                
                m_global_convexhull_cloud = boost::make_shared<base_cloud_t>();
                pcl::transformPointCloud(*m_local_convexhull_cloud, *m_global_convexhull_cloud, globalTransform());
            }
        }

        void ensureMeanVar() const{
            if(m_meanvar_invalid){
                m_meanvar_invalid = false;

                Eigen::Vector3f sx  = Eigen::Vector3f::Zero();
                Eigen::Matrix3f sxx = Eigen::Matrix3f::Zero();
                std::size_t n = size();

                typename base_cloud_t::const_iterator i;
                for(i = this->begin(); i != this->end(); i++){
                    sx  += i->getVector3fMap();
                    sxx += i->getVector3fMap() * i->getVector3fMap().transpose();
                }
                
                if(n >= 1)
                    m_mean = sx / double(n);
                else
                    m_mean = Eigen::Vector3f::Zero();
                
                if(n >= 3)
                    m_covar = (sxx - 2 * (sx * m_mean.transpose())) / double(n) + m_mean * m_mean.transpose();
                else
                    m_covar = Eigen::Matrix3f::Zero();
                
                // ensure there is a little variance in each direction
                m_covar += 0.1 * Eigen::Matrix3f::Identity();
            }
        }

        std::vector<descriptor_t> m_point_descriptors;

        // original indices of the keypoints from which the points in this
        // cloud were derived - used to generate training data for those
        // keypoints: these are not generally preserved by operations on the
        // point cloud
        std::vector<std::size_t> m_keypoint_indices;

        // original indices of the set of rejected keypoints (corresponding to
        // m_rejected_keypoints_cloud)
        std::vector<std::size_t> m_rejected_keypoint_indices;
        

        // 1 = keypoint turned out to be good, 0 = keypoint turned out to be
        // bad
        std::vector<int> m_keypoint_goodness;

        std::vector<pcl::Vertices> m_local_convexhull_verts;
        base_cloud_ptr m_local_convexhull_cloud;
        bool m_local_convexhull_invalid;
        
        // the global convex hull shares vertex indices with the local one, but
        // the transformed cloud of hull vertices is cached and used instead of
        // the non-transformed cloud
        base_cloud_ptr m_global_convexhull_cloud;
        bool m_global_convexhull_invalid;
        

        base_cloud_t m_rejected_points_cloud;

        // cache mean and covariance of the points in this cloud (in
        // POINT coordinates)
        mutable Eigen::Vector3f m_mean;
        mutable Eigen::Matrix3f m_covar;

        mutable bool m_meanvar_invalid;
};

} // namespace imgproc
} // namespace cauv 


#endif //ndef __CAUV_SONAR_SLAM_CLOUD_PART_H__
