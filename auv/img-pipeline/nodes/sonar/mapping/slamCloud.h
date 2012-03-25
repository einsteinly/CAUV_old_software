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

#ifndef __CAUV_SONAR_SLAM_CLOUD_H__
#define __CAUV_SONAR_SLAM_CLOUD_H__

#include <vector>

#include <boost/enable_shared_from_this.hpp>

#include <pcl/point_cloud.h>

#include <Eigen/StdVector>

#include <clipper.hpp> // Boost Software License

#include <debug/cauv_debug.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/KeyPoint.h>

#include "graphOptimiser.h"
#include "scanMatching.h"
#include "slamCloudPart.h"
#include "common.h"

namespace cauv{
namespace imgproc{

// return value in double-precision floating point seconds (musec precision)
static double operator-(cauv::TimeStamp const& left, cauv::TimeStamp const& right){
    const double dsecs = left.secs - right.secs;
    const double dmusecs = left.musecs - right.musecs;
    return dsecs + 1e-6*dmusecs;
}

template<typename PointT>
class SlamCloudGraph{
    public:
        // - public types
        typedef SlamCloudPart<PointT> cloud_t;
        typedef boost::shared_ptr<cloud_t> cloud_ptr;

        typedef typename cloud_t::base_cloud_t base_cloud_t;
        typedef typename base_cloud_t::Ptr base_cloud_ptr;

        typedef std::vector<cloud_ptr> cloud_vec;
        
        typedef float overlap_t;
        typedef std::multimap<overlap_t, cloud_ptr> cloud_overlap_map;
        // ++11 tuple would be useful...
        struct mat_cloud_transformed_t{
            Eigen::Matrix4f mat;
            cloud_ptr cloud;
            base_cloud_ptr transformed;
            
            // required for Matrix4f member
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            mat_cloud_transformed_t(Eigen::Matrix4f const& m, cloud_ptr cloud, base_cloud_ptr transformed)
                : mat(m), cloud(cloud), transformed(transformed){
            }
        };
        typedef std::multimap<
            score_t,
            mat_cloud_transformed_t,
            std::less<score_t>,
            Eigen::aligned_allocator< std::pair<const score_t, mat_cloud_transformed_t> >
        > cloud_constraint_map;


    public:
        // - public methods
        SlamCloudGraph()
            : m_overlap_threshold(0.3),
              m_keyframe_spacing(2),
              m_min_initial_points(10),
              m_min_initial_area(5), // m^2
              m_good_keypoint_distance(0.2),
              m_max_speed(2.0),
              m_max_considered_overlaps(3),
              m_rotation_scale(2),
              m_graph_optimisation_count(0),
              m_key_scan_locations(new KDTreeCachingCloud<PointT>()){
        }

        void reset(){
            m_key_scans.clear();
            m_all_scans.clear();
            m_key_constraints.clear();
            m_graph_optimisation_count = 0;
            m_key_scan_locations->clear();
        }

        int graphOptimisationsCount() const{
            return m_graph_optimisation_count;
        }

        void setParams(float overlap_threshold,
                       float keyframe_spacing,
                       float min_initial_points,
                       float good_keypoint_distance){
            if(overlap_threshold > 0.5){
                warning() << "invalid overlap threshold" << overlap_threshold
                          << "(>0.5) will be ignored";
                overlap_threshold = m_overlap_threshold;
            }
            m_overlap_threshold = overlap_threshold;
            m_keyframe_spacing = keyframe_spacing;
            m_min_initial_points = min_initial_points;
            m_good_keypoint_distance = good_keypoint_distance;
        }

        cloud_vec const& keyScans() const{
            return m_key_scans;
        }

        location_vec const& allScans() const{
            return m_all_scans;
        }

        /* Guess a transformation for a time based on simple extrapolation of
         * the previous transformations.
         * Returned transformation is in the global coordinate system
         */
        Eigen::Matrix4f guessTransformationAtTime(TimeStamp const& t) const{
            switch(m_all_scans.size()){
                case 0:
                    return Eigen::Matrix4f::Identity();
                case 1:
                    assert(!m_all_scans.back()->relativeTo());
                    return m_all_scans.back()->globalTransform();
                // this seems to harm performance!
                /*default:{
                    // !!! TODO: use more than one previous point for smooth
                    // estimate?
                    location_vec::const_reverse_iterator i = m_all_scans.rbegin();
                    const Eigen::Vector3f p1 = (*i)->globalTransform().block<3,1>(0,3);
                    const TimeStamp t1 = (*i)->time();
                    const Eigen::Vector3f p2 = (*++i)->globalTransform().block<3,1>(0,3);
                    const TimeStamp t2 = (*i)->time();
                    const Eigen::Vector3f p = p1 + (p1-p2)*(t-t1)/(t1-t2);

                    const float frac_speed = std::fabs((p-p1).norm() / (t-t1)) / m_max_speed;
                    Eigen::Matrix4f r = Eigen::Matrix4f::Identity();
                    r.block<3,1>(0,3) = p1 + (p1-p2)*(t-t1)/(t1-t2);
                    if(frac_speed >= 1.0){
                        warning() << "predicted motion > max speed (x"
                                  << frac_speed << "), will throttle";
                        r.block<3,1>(0,3) = p1 + (1.0/frac_speed) * (p1-p2)*(t-t1)/(t1-t2);
                    }
                    if((p-p1).norm() > m_keyframe_spacing){
                        warning() << "predicted motion > keyframe spacing, will throttle";
                        r.block<3,1>(0,3) = p1 + m_keyframe_spacing * (p1-p2).normalized();
                    }
                    return r;
                }*/
                default:
                    return m_all_scans.back()->globalTransform();
            }
        }

        /* Match scan 'p' to the map, return the confidence in the match.
         * 'transformation' is set to the global transformation of the new
         * scan.
         */
        float registerScan(cloud_ptr p,
                           Eigen::Matrix4f const& guess,
                           PairwiseMatcher<PointT> const& m,
                           GraphOptimiser const& graph_optimiser,
                           Eigen::Matrix4f& transformation){
            if(!m_key_scans.size()){
                if(cloudIsGoodEnoughForInitialisation(p)){
                    m_key_scans.push_back(p);
                    m_all_scans.push_back(p);
                    transformation = Eigen::Matrix4f::Identity();
                    p->setRelativeToNone();
                    p->setRelativeTransform(transformation);
                    return 1.0f;
                }else{
                    debug() << "cloud not good enough for initialisation";
                    return 0.0f;
                }
            }

            if(p->size() < 3){
                warning() << "too few points in cloud (<3) to even consider it";
                return 0.0f;
            }

            const cloud_overlap_map overlaps = overlappingClouds(p, guess);
            cloud_constraint_map transformations;

            Eigen::Matrix4f relative_transformation;
            float r = 0.0f;

            if(overlaps.size() == 0){
                error() << "new scan falls outside map!";
                return 0;
            }
            // for each overlap (up to m_max_considered_overlaps), in order of
            // goodness, align this new scan to the overlapping one, and save
            // the resulting transformation:
            int limit = m_max_considered_overlaps;
            int succeeded_match = 0;
            typename cloud_overlap_map::const_iterator i;
            debug() << "matching to overlapping scans...";
            int failed_match = 0;
            for(i = overlaps.begin(); i != overlaps.end() && limit; i++, limit--){
                try{
                    cloud_ptr map_cloud = i->second;
                    base_cloud_ptr transformed = boost::make_shared<base_cloud_t>();
                    float score = m.transformcloudToMatch(
                        map_cloud, p, guess, relative_transformation, transformed
                    );
                    transformations.insert(std::make_pair(
                        score,
                        mat_cloud_transformed_t(
                            relative_transformation, map_cloud, transformed
                        )
                    ));
                    succeeded_match++;
                }catch(PairwiseMatchException& e){
                    failed_match++;
                    continue;
                }
            }
            debug() << succeeded_match << "matches succeeded"
                    << failed_match << "failed";
            
            # if 0
            // remove scans that imply moving too fast?
                ... code previously used to do this: (transformation is
                relative_transformation * map_cloud->relativeTransform() for
                each map_cloud in transformations)

                Eigen::Matrix4f last_transform = m_all_scans.back()->globalTransform();
                const float speed = (transformation - last_transform).block<3,1>(0,3).norm() /
                                    (p->time() - m_all_scans.back()->time());
                if(speed > m_max_speed){
                    warning() << "match implies moving too fast: ignoring ("
                              << speed << "/" << m_max_speed << ")";
                    return 0;
                }
            #endif // 0

            if(transformations.size() == 0){
                error() << "no overlapping scans matched!";
                return 0;
            }

            debug() << transformations.size() << "overlapping scans matched"
                    << "scores"
                    << transformations.rbegin()->first << "--"
                    << transformations.begin()->first;

            // Choose the best *score* (not overlap) out of these matches, and use as the
            // parent for this scan:
            const cloud_ptr       parent_map_cloud   = transformations.rbegin()->second.cloud;
            const Eigen::Matrix4f rel_transformation = transformations.rbegin()->second.mat;
            const base_cloud_ptr  transformed        = transformations.rbegin()->second.transformed;
            r = transformations.rbegin()->first;

            p->setRelativeTransform(rel_transformation);
            p->setRelativeTo(parent_map_cloud);
            // set the returned transformation (if there are other constraints,
            // then we set this again after graph optimisation)
            transformation = rel_transformation * parent_map_cloud->globalTransform();
            
            // this scan is a key scan if it is more than a minimum distance
            // from other key-scans
            const Eigen::Vector3f xyr = xytFromScanTransformation(p->globalTransform());
            const PointT xyr_space_loc = PointT(xyr[0], xyr[1], xyr[2]);
            const float squared_keyframe_spacing = m_keyframe_spacing * m_keyframe_spacing;
            if(m_key_scan_locations->nearestSquaredDist(xyr_space_loc) > squared_keyframe_spacing){
                // key scans are transformed to global coordinate
                // frame
                p->setRelativeTransform(p->globalTransform());
                p->setRelativeToNone();
                m_key_scans.push_back(p);
                m_key_scan_locations->push_back(PointT(xyr_space_loc));
                m_all_scans.push_back(p);
                debug() << "key frame at"
                        << transformation.block<3,1>(0, 3).transpose()
                        << ":" << std::sqrt(squared_keyframe_spacing)
                        << "from previous scans";

                // If this is a new key scan, we need to re-run the graph
                // optimiser:

                // convert relative positions into incremental position
                // constraints
                constraint_vec new_constraints = addConstraintsFromTransformations(
                    p, transformations.rbegin(), transformations.rend()
                ); 
                
                // do the optimisation, hint at which constraints are new so
                // they can be prioritised
                graph_optimiser.optimiseGraph(m_key_constraints, new_constraints);
                m_graph_optimisation_count++;
            }else{
                // discard all the point data for non-key scans
                m_all_scans.push_back(boost::make_shared<SlamCloudLocation>(p));
                debug() << "non-key frame at"
                        << transformation.block<3,1>(0,3).transpose();
            }


            // Set keypoint goodness for training:
            // !!! TODO: this should ideally include more than just points from the direct parent

            // 'transformed' is in the same coordinate frame as
            // parent_map_cloud, so we can easily find nearest neighbors in
            // map_cloud to use as a measure of how good the keypoints  were:
            std::vector<int>   pt_indices(1);
            std::vector<float> pt_squared_dists(1);

            int ngood = 0;
            int nbad = 0;
            for(size_t i=0; i < transformed->size(); i++){
                if(parent_map_cloud->nearestKSearch((*transformed)[i], 1, pt_indices, pt_squared_dists) > 0 &&
                   pt_squared_dists[0] < m_good_keypoint_distance){
                    p->keyPointGoodness()[p->ptIndices()[i]] = 1;
                    ngood++;
                }else{
                    p->keyPointGoodness()[p->ptIndices()[i]] = 0;
                    nbad++;
                }
            }
            debug() << float(ngood)/(nbad+ngood) << "keypoints proved good";

            return r;
        }

    private:
        // - private methods
        /* Return all cloud parts in the map that overlap with p by more than
         * m_overlap_threshold.
         */
        cloud_overlap_map overlappingClouds(cloud_ptr p) const{
            cloud_overlap_map r;
            foreach(cloud_ptr m, m_key_scans){
                float overlap = overlapPercent(m, p);
                if(overlap > m_overlap_threshold)
                    r.insert(typename cloud_overlap_map::value_type(overlap, m));
            }
            return r;
        }

        /* overload to find overlaps at a different transformation to the
         * current one (additional transformation is applied last, ie:
         * global_point = p->relativeTo().relativeTransform() *  p->relativeTransform() * additional_transform * cloud_point;
         */
        cloud_overlap_map overlappingClouds(cloud_ptr p, Eigen::Matrix4f const& additional_transform){
            Eigen::Matrix4f saved_transform = p->relativeTransform();
            p->setRelativeTransform(saved_transform * additional_transform);
            cloud_overlap_map r = overlappingClouds(p);
            p->setRelativeTransform(saved_transform);
            return r;
        }

        /* Judge degree of overlap: new parts are added to the map when the
         * overlap with existing parts is less than m_overlap_threshold
         */
        float overlapPercent(cloud_ptr a, cloud_ptr b) const{
            assert(a->size() != 0 && b->size() != 0);

            // !!! TODO: cache convex hulls with point clouds
            std::vector<pcl::Vertices> a_polys;
            std::vector<pcl::Vertices> b_polys;
            base_cloud_ptr a_points;
            base_cloud_ptr b_points;
            a->getGlobalConvexHull(a_points, a_polys);
            b->getGlobalConvexHull(b_points, b_polys);

            assert(a_polys.size() == 1);
            assert(b_polys.size() == 1);

            ClipperLib::Polygon clipper_poly_a;
            ClipperLib::Polygon clipper_poly_b;
            ClipperLib::Polygons solution;

            //clipperlib works in 64-bit fixed point, so scale our 1m=1
            // floating point data up by 1000 to 1mm=1

            clipper_poly_a.reserve(a_polys[0].vertices.size());
            foreach(uint32_t i, a_polys[0].vertices)
                clipper_poly_a.push_back(ClipperLib::IntPoint((*a_points)[i].x*1000,(*a_points)[i].y*1000));

            clipper_poly_b.reserve(b_polys[0].vertices.size());
            foreach(uint32_t i, b_polys[0].vertices)
                clipper_poly_b.push_back(ClipperLib::IntPoint((*b_points)[i].x*1000,(*b_points)[i].y*1000));

            ClipperLib::Clipper c;
            c.AddPolygon(clipper_poly_a, ClipperLib::ptSubject);
            c.AddPolygon(clipper_poly_b, ClipperLib::ptClip);
            c.Execute(ClipperLib::ctIntersection, solution);

            if(solution.size() != 0){
                // return area of overlap as fraction of union of areas of input
                // polys
                // areas can be negative due to vertex order, hence the fabs-ing
                const double union_area = std::fabs(ClipperLib::Area(solution[0]));
                const double a_area = std::fabs(ClipperLib::Area(clipper_poly_a));
                const double b_area = std::fabs(ClipperLib::Area(clipper_poly_b));

                debug(3) << "overlap pct =" << 1e-6*union_area
                         << "/ (" << -1e-6*a_area << "+" << -1e-6*b_area << ") ="
                         << union_area / (a_area + b_area) << "(m^2)";
                return union_area / (a_area + b_area);
            }else{
                return 0;
            }
        }
        
        // return area of conveh hull in m^2
        static float area(cloud_ptr a){
           assert(a->size() != 0);

            // !!! TODO: cache convex hulls with point clouds
            std::vector<pcl::Vertices> a_polys;
            base_cloud_ptr a_points;
            a->getGlobalConvexHull(a_points, a_polys);

            assert(a_polys.size() == 1);

            ClipperLib::Polygon clipper_poly_a;

            //clipperlib works in 64-bit fixed point, so scale our 1m=1
            // floating point data up by 1000 to 1mm=1
            clipper_poly_a.reserve(a_polys[0].vertices.size());
            foreach(uint32_t i, a_polys[0].vertices)
                clipper_poly_a.push_back(ClipperLib::IntPoint((*a_points)[i].x*1000,(*a_points)[i].y*1000));

            return 1e-6 * std::fabs(ClipperLib::Area(clipper_poly_a));
        }

        /* judge how good initial cloud is by a combination of the number of
         * features, and how well it matches with itself, or something.
         */
        bool cloudIsGoodEnoughForInitialisation(cloud_ptr p) const{
            return (p->size() > m_min_initial_points) &&
                   (area(p) > m_min_initial_area);
        }

        /* Convert 4d affine transformation of a scan origin into 3D (x, y,
         * scaled rotation) space in which a constant distance metric is used
         * to decide on placement of new key scans
         */
        Eigen::Vector3f xytFromScanTransformation(Eigen::Matrix4f const& a) const{
            const Eigen::Matrix2f r = a.block<2,2>(0, 0);
            const Eigen::Vector2f t = a.block<2,1>(0, 3);

            const Eigen::Vector2f tmp = r * Eigen::Vector2f(1,0);
            const float rz = std::atan2(tmp[1], tmp[0]);
            return Eigen::Vector3f(t[0], t[1], m_rotation_scale * rz);
        }

        /* add constraints based on relative transformations, return the added
         * constraints as well as adding them to key_constraints
         */
        template<typename IterT>
        constraint_vec addConstraintsFromTransformations(
            cloud_ptr p, IterT it, IterT end_it
        ){
            constraint_vec r;
            
            while(it != end_it){
                r.push_back(SlamCloudLocation::addConstraintBetween(
                    p, it->second.cloud, it->second.mat
                ));
                it++;
            }
            
            m_key_constraints.insert(m_key_constraints.end(), r.begin(), r.end());
            return r;
        }


        // - private data
        float m_overlap_threshold; // a fraction (0--0.5)
        float m_keyframe_spacing;  // in metres
        float m_min_initial_points; // for first keyframe
        float m_min_initial_area;   //
        /* new keypoints with nearest nieghbours better than this are
         * considered good for training: */
        float m_good_keypoint_distance;
        float m_max_speed;
        int m_max_considered_overlaps;
        // Used to map rotation to a Euclidean distance when deciding to place
        // a new keyframe if:
        // m_rotation_scale * (rotation_A - rotation_B) >= m_keyframe_spacing
        float m_rotation_scale;
        
        // +1 each time graph optimiser is run - i.e. this number changes
        // whenever everything might have moved
        int m_graph_optimisation_count;

        // TODO: to remain efficient there MUST be a way of searching for
        // SlamCloudParts near a location without iterating through all nodes
        // (kdtree probably)
        cloud_vec m_key_scans;
        
        // each point in this cloud is a key scan (same order as m_key_scans):
        // x,y,z map to x,y,m_rotation_scale*rotation 
        boost::shared_ptr< KDTreeCachingCloud<PointT> > m_key_scan_locations;
        
        
        // similarly, this will need some thought to scale well
        constraint_vec m_key_constraints;

        // for these scans, all data apart from the time and relative location
        // is discarded
        location_vec m_all_scans;
};












} // namespace cauv
} // namespace imgproc


#if 0
// - deprecated

#include <pcl/surface/concave_hull.h>
#include <pcl/filters/crop_hull.h>

namespace cauv{
namespace imgproc{

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

#endif // 0

#endif //ndef __CAUV_SONAR_SLAM_CLOUD_H__
