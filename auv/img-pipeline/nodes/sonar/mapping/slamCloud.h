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
#include <fstream>
#include <ios>

#include <sys/types.h>
#include <unistd.h>

#include <boost/enable_shared_from_this.hpp>

#include <pcl/point_cloud.h>

#include <Eigen/StdVector>

#include <polyclipping/clipper.hpp> // Boost Software License

#include <debug/cauv_debug.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/KeyPoint.h>
#include <generated/types/floatXYZ.h>
#include <common/msg_classes/north_east_depth.h>
#include <utility/string.h>

#include "graphOptimiser.h"
#include "scanMatching.h"
#include "slamCloudPart.h"
#include "common.h"
#include "stuff.h"

namespace cauv{
namespace imgproc{

struct CloudGraphParams{
    // default constructor sets sensible defaults:
    CloudGraphParams() :
        overlap_threshold(0.3),
        keyframe_spacing(2),
        min_initial_points(10),
        min_initial_area(5), // m^2 !!! should be a function of range
        good_keypoint_distance(0.2),
        max_speed(1.0),
        max_considered_overlaps(3),
        min_scan_consensus(2),
        scan_consensus_tolerance(0.3),
        rotation_scale(4),
        persistance_dir("."){
    }

    float overlap_threshold; // a fraction (0--1.0)
    float keyframe_spacing;  // in metres
    float min_initial_points; // for first keyframe
    float min_initial_area;   //
    /* new keypoints with nearest neighbours better than this are
     * considered good for training:
     */
    float good_keypoint_distance;
    float max_speed;
    int max_considered_overlaps;
    
    unsigned min_scan_consensus; // n >= 1
    float scan_consensus_tolerance; // in metres
    
    // Used to map rotation to a Euclidean distance when deciding to place a
    // new keyframe if:
    // rotation_scale * (rotation_A - rotation_B) >= keyframe_spacing
    float rotation_scale;
    
    std::string persistance_dir;
};

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

        typedef typename cloud_constraint_map::const_iterator ccmap_const_iter;


    public:
        // - public methods
        SlamCloudGraph(CloudGraphParams const& params = CloudGraphParams())
            : m_params(params),
              m_graph_optimisation_count(0),
              m_dump_pose_history(CAUV_DUMP_POSE_GRAPH),
              m_key_scans(),
              m_key_scan_indicies(),
              m_key_scan_locations(new KDTreeCachingCloud<PointT>()),
              m_n_passed_good(0),
              m_n_passed_bad(0),
              m_n_failed_good(0),
              m_n_failed_bad(0){
        }

        virtual ~SlamCloudGraph(){
            info() << "SlamCloudGraph lifetime statistics:\n"
                   << "\tpassed weight test, were good:" << m_n_passed_good << "\n"
                   << "\tpassed weight test, were bad :" << m_n_passed_bad  << "\n"
                   << "\tfailed weight test, were good:" << m_n_failed_good << "\n"
                   << "\tfailed weight test, were bad :" << m_n_failed_bad;
            unsigned total = m_n_passed_good + m_n_passed_bad + m_n_failed_good + m_n_failed_bad;
            info() << "Total keypoints processed:" << total;
            info() << "Classifier Wrong Reject:" << 100*float(m_n_failed_good) / total << "%";
            info() << "Classifier Wrong Accept:" << 100*float(m_n_passed_bad) / total << "%";
        }

        void reset(){
            m_key_scans.clear();
            m_key_scan_indicies.clear();
            m_all_scans.clear();
            m_key_constraints.clear();
            m_graph_optimisation_count = 0;
            m_key_scan_locations->clear();
            m_n_passed_good = 0;
            m_n_passed_bad = 0;
            m_n_failed_good = 0;
            m_n_failed_bad = 0;
        }

        int graphOptimisationsCount() const{
            return m_graph_optimisation_count;
        }

        void setParams(CloudGraphParams params){
            m_params = params;
        }

        void setParams(float overlap_threshold,
                       float keyframe_spacing,
                       float min_initial_points,
                       float good_keypoint_distance,
                       int max_considered_overlaps){
            if(overlap_threshold > 1.0){
                warning() << "invalid overlap threshold" << overlap_threshold
                          << "(>1.0) will be ignored";
                overlap_threshold = m_params.overlap_threshold;
            }
            m_params.overlap_threshold = overlap_threshold;
            m_params.keyframe_spacing = keyframe_spacing;
            m_params.min_initial_points = min_initial_points;
            m_params.good_keypoint_distance = good_keypoint_distance;
            m_params.max_considered_overlaps = max_considered_overlaps;
        }

        cloud_vec const& keyScans() const{
            return m_key_scans;
        }

        location_vec const& allScans() const{
            return m_all_scans;
        }

        /* Guess a transformation for a time based on simple extrapolation of
         * the previous transformations.
         * Returned transformation is in the global coordinate system.
         */
        Eigen::Matrix4f guessTransformationAtTime(
            TimeStamp const& t,
            Eigen::Matrix4f const& rotation_guess_relative_to_last_scan
        ) const{
            switch(m_all_scans.size()){
                case 0:
                    return rotation_guess_relative_to_last_scan;
                case 1:
                    assert(!m_all_scans.back()->relativeTo());
                    return m_all_scans.back()->globalTransform() * rotation_guess_relative_to_last_scan;
                default:{
                    // !!! TODO: use more than one previous point for smooth
                    // estimate?
                    Eigen::Matrix4f r = m_all_scans.back()->globalTransform() * rotation_guess_relative_to_last_scan;
                    
                    location_vec::const_reverse_iterator i = m_all_scans.rbegin();
                    const Eigen::Vector3f p1 = (*i)->globalTransform().block<3,1>(0,3);
                    const TimeStamp t1 = (*i)->time();
                    Eigen::Vector3f p2 = (*++i)->globalTransform().block<3,1>(0,3);
                    TimeStamp t2 = (*i)->time();
                    while(t2.secs == t1.secs && t1.musecs == t2.musecs && ++i != m_all_scans.rend()){
                        p2 = (*++i)->globalTransform().block<3,1>(0,3);
                        t2 = (*i)->time();
                    }
                    if(t2.secs == t1.secs && t1.musecs == t2.musecs){
                        debug() << "all scans have the same timestamp!";
                        return r;
                    }
                    const Eigen::Vector3f p = p1 + (p1-p2)*(t-t1)/(t1-t2);
                    debug(3) << "p1:" << p1.transpose() << "t1:" << t1
                             << "p2:" << p2.transpose() << "t2:" << t2;
                    if(t-t1 < 1e-4){
                        debug() << "guess transformation in < 0.1 ms!";
                        return r;
                    }
                    const float frac_speed = std::fabs((p-p1).norm() / (t-t1)) / m_params.max_speed;
                    r.block<3,1>(0,3) = p;
                    if(frac_speed >= 1.0){
                        warning() << "predicted motion > max speed (x"
                                  << frac_speed << "), will throttle";
                        r.block<3,1>(0,3) = p1 + (1.0/frac_speed) * (p1-p2)*(t-t1)/(t1-t2);
                    }
                    if((p-p1).norm() > m_params.keyframe_spacing){
                        warning() << "predicted motion > keyframe spacing, will throttle";
                        r.block<3,1>(0,3) = p1 + m_params.keyframe_spacing * (p1-p2).normalized();
                    }
                    return r;
                }
                //default:
                //    return m_all_scans.back()->globalTransform();
            }
        }

        floatXYZ speed() const{
            if(m_all_scans.size() >= 2){
                location_vec::const_reverse_iterator i = m_all_scans.rbegin();
                const Eigen::Vector3f p1 = (*i)->globalTransform().block<3,1>(0,3);
                const TimeStamp t1 = (*i)->time();
                const Eigen::Vector3f p2 = (*++i)->globalTransform().block<3,1>(0,3);
                const TimeStamp t2 = (*i)->time();
                const Eigen::Vector3f speed = (p1-p2) / (t1-t2);
                return floatXYZ(speed(0), speed(1), speed(2));
            }else{
                return floatXYZ(0,0,0);
            }
        }

        NorthEastDepth position(float const& with_origin_yaw_radians=0, float const& at_depth=0){
            if(m_all_scans.size()){
                Eigen::Vector3f xytheta = xyThetaFrom4DAffine(m_all_scans.back()->globalTransform());
                const float north = std::sin(with_origin_yaw_radians) * xytheta(0) +
                                    std::cos(with_origin_yaw_radians) * xytheta(1);
                const float east = std::cos(with_origin_yaw_radians) * xytheta(0) -
                                   std::sin(with_origin_yaw_radians) * xytheta(1);
                return NorthEastDepth(north, east, at_depth);
            }else{
                return NorthEastDepth(0, 0, at_depth);
            }
        }

        Eigen::Matrix4f lastTransform() const{
            if(m_all_scans.size()){
                return m_all_scans.back()->globalTransform();
            }else{
                return Eigen::Matrix4f::Identity();
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
                    m_key_scan_indicies[p] = m_key_scans.size();
                    m_key_scans.push_back(p);
                    m_all_scans.push_back(p);
                    //transformation = Eigen::Matrix4f::Identity();
                    transformation = guess;
                    p->setRelativeToNone();
                    p->setRelativeTransform(transformation);
                    const Eigen::Vector3f xyr = xyScaledTFromMat(p->globalTransform());
                    m_key_scan_locations->push_back(PointT(xyr[0], xyr[1], xyr[2]));
                    return 1.0f;
                }else{
                    transformation = Eigen::Matrix4f::Zero();
                    return 0.0f;
                }
            }

            if(p->size() < 3){
                warning() << "too few points in cloud (<3) to even consider it";
                transformation = Eigen::Matrix4f::Zero();
                return 0.0f;
            }

            const cloud_overlap_map overlaps = overlappingClouds(p, guess);
            cloud_constraint_map transformations;

            Eigen::Matrix4f relative_transformation;
            float r = 0.0f;

            if(overlaps.size() == 0){
                error() << "new scan falls outside map! guess was:" << guess;
                transformation = guess;
                return 0;
            }
            // for each overlap (up to m_params.max_considered_overlaps), in order of
            // goodness, align this new scan to the overlapping one, and save
            // the resulting transformation:

            // !!! scores from matching are are  0 (worst) -- 1 (best)

            int limit = m_params.max_considered_overlaps;
            int succeeded_match = 0;
            int failed_match = 0;
            typename cloud_overlap_map::const_iterator i;
            for(i = overlaps.begin(); i != overlaps.end() && limit; i++, limit--){
                try{
                    cloud_ptr map_cloud = i->second;
                    base_cloud_ptr transformed = boost::make_shared<base_cloud_t>();
                    relative_transformation = Eigen::Matrix4f::Identity();
                    float score = m.transformcloudToMatch(
                        map_cloud, p, guess, relative_transformation, transformed
                    );
                    float age_penalty = 1.0 + std::log(1.0 + (p->time() - map_cloud->time()));
                    transformations.insert(std::make_pair(
                        score / age_penalty,
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
            
            /* this is mostly redundant now consensus is implemented

            // remove scans that imply moving too fast:
            cloud_constraint_map transformations_speed_checked;            
            
            Eigen::Matrix4f last_transform = m_all_scans.back()->globalTransform();
            ccmap_const_iter ti;
            for(ti = transformations.begin(); ti != transformations.end(); ti++){
                const Eigen::Matrix4f tr = ti->second.mat * ti->second.cloud->globalTransform();
                const float speed = (tr - last_transform).block<3,1>(0,3).norm() /
                                    (p->time() - ti->second.cloud->time());
                if(speed > m_params.max_speed){
                    warning() << "match implies moving too fast: ignoring ("
                              << speed << "/" << m_params.max_speed << ")";
                }else{
                    //debug() << "match implies moving at" << speed / m_params.max_speed << "x maximum speed";
                    transformations_speed_checked.insert(*ti);
                }
            }

            if(transformations_speed_checked.size() == 0){
                error() << "no overlapping scans matched!";
                if(transformations.size()){
                    // choose the best to return anyway:
                    transformation = transformations.rbegin()->second.cloud->globalTransform() * transformations.rbegin()->second.mat;
                }else{
                    transformation = guess;
                }
                return 0;
            }
            transformations = transformations_speed_checked;
            */
            if(!transformations.size()){
                error() << "no overlapping scans matched!";
                transformation = guess;
                return 0;
            }

            debug() << transformations.size() << "overlapping scans matched"
                    << "scores"
                    << transformations.rbegin()->first << "--"
                    << transformations.begin()->first;
            
            /*debug(5) << "transformations to map scans:";
            for(ti = transformations.begin(); ti != transformations.end(); ti++){
                debug(5) << "score=" << ti->first << ", relative transformation=\n" << ti->second.mat;
                const Eigen::Vector3f xyr = xyThetaFrom4DAffine(ti->second.mat);
                debug(5) << "pos=" << xyr[0] << "," << xyr[1] << ": rotation=" << xyr[2]*180/M_PI << "deg";
            }*/

            std::vector<ccmap_const_iter> consensus_set;
            ccmap_const_iter best = findBestConsensusSet(
                transformations.begin(), transformations.end(), consensus_set
            );

            if(best == transformations.end()){
                warning() << "insufficient consensus";
                best = consensus_set.back();
                transformation = guess;                
                return 0;
            }

            const cloud_ptr       parent_map_cloud   = best->second.cloud;
            const Eigen::Matrix4f rel_transformation = best->second.mat;
            const base_cloud_ptr  transformed        = best->second.transformed;
            r = best->first; // !!! score should include degree of consensus
            
            // only consider the consensus transformations
            cloud_constraint_map consensus_transformations;            
            foreach(ccmap_const_iter const& c, consensus_set)
                consensus_transformations.insert(*c);
            
            transformations = consensus_transformations;

            // ---- best, and consensus_set content iterators now invalid ----

            p->setRelativeTransform(rel_transformation);
            p->setRelativeTo(parent_map_cloud);
            
            // this scan is a key scan if it is more than a minimum distance
            // from other key-scans
            if(shouldBeKeyScan(p)){
                addKeyScan(p, transformations, graph_optimiser);
                saveKeyScan(p, m_key_scans.size()-1);
            }else{
                // discard all the point data for non-key scans
                m_all_scans.push_back(boost::make_shared<SlamCloudLocation>(p));
                debug() << "non-key frame at"
                        << transformation.block<3,1>(0,3).transpose();
                saveIntermediatePose(p, m_all_scans.size()-1);
            }
            transformation = p->globalTransform();

            // Set keypoint goodness for training:
            // !!! TODO: this should ideally include more than just points from the direct parent

            // 'transformed' is in the same coordinate frame as
            // parent_map_cloud, so we can easily find nearest neighbors in
            // map_cloud to use as a measure of how good the keypoints  were:
            std::vector<int>   pt_indices(1);
            std::vector<float> pt_squared_dists(1);

            int n_passed_good = 0; // passed the weight test, and turned out to be good
            int n_passed_bad = 0;  // passed the weight test, but turned out to be bad
            int n_failed_good = 0; // failed the weight test, but would have been good
            int n_failed_bad = 0;  // failed the weight test, and would have been bad

            // first, the points that passed the weight test:
            for(size_t i=0; i < transformed->size(); i++){
                if(parent_map_cloud->nearestKSearch((*transformed)[i], 1, pt_indices, pt_squared_dists) > 0 &&
                   pt_squared_dists[0] < m_params.good_keypoint_distance){
                    p->keyPointGoodness()[p->ptIndices()[i]] = 1;
                    n_passed_good++;
                }else{
                    p->keyPointGoodness()[p->ptIndices()[i]] = 0;
                    n_passed_bad++;
                }
            }

            // now transform the ones that failed into the parent coordinate
            // system:
            base_cloud_t transformed_rejected;
            pcl::transformPointCloud(p->rejectedPoints(), transformed_rejected, p->relativeTransform());

            for(size_t i = 0; i < transformed_rejected.size(); i++){
                if(parent_map_cloud->nearestKSearch(transformed_rejected[i], 1, pt_indices, pt_squared_dists) > 0 &&
                   pt_squared_dists[0] < m_params.good_keypoint_distance){
                    p->keyPointGoodness()[p->rejectedPtIndices()[i]] = 1;
                    n_failed_good++;
                }else{
                    p->keyPointGoodness()[p->rejectedPtIndices()[i]] = 0;
                    n_failed_bad++;
                }
            }

            const int total = n_passed_good + n_passed_bad + n_failed_good + n_failed_bad;

            m_n_passed_good += n_passed_good;
            m_n_passed_bad  += n_passed_bad;
            m_n_failed_good += n_failed_good;
            m_n_failed_bad  += n_failed_bad;
            
            info() << "classifier statistics:\n"
                   << "\tpass,good =" << n_passed_good << "\t= " << 100*float(n_passed_good)/total << "%\n"
                   << "\tpass,bad  =" << n_passed_bad  << "\t= " << 100*float(n_passed_bad)/total << "%\n"
                   << "\tfail,good =" << n_failed_good << "\t= " << 100*float(n_failed_good)/total << "%\n"
                   << "\tfail,bad  =" << n_failed_bad  << "\t= " << 100*float(n_failed_bad)/total << "%\n";

            if(m_dump_pose_history)
                dumpPoseHistory();

            return r;
        }

    private:
        // - private methods
        bool shouldBeKeyScan(cloud_ptr p){
            const Eigen::Vector3f xyr = xyScaledTFromMat(p->globalTransform());
            const PointT xyr_space_loc = PointT(xyr[0], xyr[1], xyr[2]);
            const float squared_keyframe_spacing = m_params.keyframe_spacing * m_params.keyframe_spacing;
            if(m_key_scan_locations->nearestSquaredDist(xyr_space_loc) > squared_keyframe_spacing)
                return true;
            return false;
        }

        void addKeyScan(cloud_ptr p,
                        cloud_constraint_map const& transformations,
                        GraphOptimiser const& graph_optimiser){
            const Eigen::Matrix4f transformation = p->globalTransform();
            const Eigen::Vector3f xyr = xyScaledTFromMat(transformation);
            const PointT xyr_space_loc = PointT(xyr[0], xyr[1], xyr[2]);
            // key scans are transformed to global coordinate
            // frame
            p->setRelativeTransform(transformation);
            p->setRelativeToNone();
            m_key_scan_indicies[p] = m_key_scans.size();
            m_key_scans.push_back(p);
            m_key_scan_locations->push_back(PointT(xyr_space_loc));
            m_all_scans.push_back(p);
            debug() << "key frame at:"
                    << Eigen::Vector3f(transformation.block<3,1>(0, 3)).transpose()
                    << "m from previous scans";

            // need to re-run the graph optimiser:

            // convert relative positions into incremental position
            // constraints
            constraint_vec new_constraints = addConstraintsFromTransformations(
                p, transformations.rbegin(), transformations.rend()
            ); 
            
            // do the optimisation, hint at which constraints are new so
            // they can be prioritised
            graph_optimiser.optimiseGraph(m_key_constraints, new_constraints);
            m_graph_optimisation_count++;
            debug() << "post-optimisation, key frame at:"
                    << Eigen::Vector3f(transformation.block<3,1>(0, 3)).transpose();
            
            // update the spatially-sorted datastructure of key-scan locations
            _updateKeyScanLocations();

            saveKeyFramePositions();
        }

        void saveKeyScan(cloud_ptr p, std::size_t id){
            std::string kfn = mkStr() << m_params.persistance_dir << "/" << id << ".keyframe";
            std::ofstream kf(kfn.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
            p->saveToFile(kf);
            kf.close();
            
            std::string cfn = mkStr() << m_params.persistance_dir << "/" << id << ".constraints";
            std::ofstream cf(cfn.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
            cf << std::flush;
            cf.close();

            std::string cpn = mkStr() << m_params.persistance_dir << "/" << id << ".relposes";
            std::ofstream cp(cpn.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);
            cp << std::flush;
            cp.close();
            
            SlamCloudLocation::rel_pose_vec constraints = p->constraints();
            location_vec constrained_to = p->constrainedTo();
            assert(constrained_to.size() == constraints.size());

            for(std::size_t i = 0; i < constraints.size(); i++){
                const std::size_t parent_id = m_key_scan_indicies[constrained_to[i]];
                std::string fname = mkStr() << m_params.persistance_dir << "/" << parent_id<< ".constraints";
                std::ofstream f(fname.c_str(), std::ios::out | std::ios::binary | std::ios::app);
            
                f.write((char*)&id, id);
                constraints[i].saveToFile(f);
                
                f.close();
            }
        }

        static void saveMat(std::ofstream& f, Eigen::Matrix4f const& m){
            for(int i = 0; i < 4; i++)
                for(int j = 0; j < 4; j++)
                    f.write(reinterpret_cast<const char*>(&m(i,j)), sizeof(m(i,j)));
        }

        void saveIntermediatePose(cloud_ptr p, std::size_t id){
            std::size_t parent_id = m_key_scan_indicies[p->relativeTo()];
            std::string cpn = mkStr() << m_params.persistance_dir << "/" << parent_id << ".relposes";
            std::ofstream cp(cpn.c_str(), std::ios::out | std::ios::binary | std::ios::app);
            cp.write((char*)&id, sizeof(id));
            TimeStamp t = p->time();
            cp.write((char*)&t, sizeof(t));
            saveMat(cp, p->relativeTransform());
            cp.close();
        }

        void saveKeyFramePositions(){
            const TimeStamp timestamp = now();
            std::string fname = mkStr() << m_params.persistance_dir << "/" << timestamp.secs << (timestamp.musecs / 1000) << ".nodes";
            std::ofstream f(fname.c_str(), std::ios::out | std::ios::binary | std::ios::trunc);

            for(std::size_t i = 0; i < m_key_scans.size(); i++)
                saveMat(f, m_key_scans[i]->globalTransform());

            f.close();
        }
        

        /* Find the best consensus amongst a set of possible transformations
         *
         * !!!! TODO: use consensus-weighted score instead?
         *
         * When adding a key-scan we also need to know which matching scans
         * didn't agree, so they can be ignored, hence the whole consensus set
         * set is returned (at little additional cost).
         * In the case that no consensus is found, the best supported match is
         * still returned as the first iterator in best_consensus_set.
         */
        ccmap_const_iter findBestConsensusSet(ccmap_const_iter b,
                                              ccmap_const_iter e,
                                              std::vector<ccmap_const_iter>& best_consensus_set) const{
            const float sq_tolerance = m_params.scan_consensus_tolerance * m_params.scan_consensus_tolerance;
            std::vector<ccmap_const_iter> consensus_set;
            ccmap_const_iter i = b;
            ccmap_const_iter m = b;
            ccmap_const_iter r = e;

            best_consensus_set.clear();
            
            std::vector<Eigen::Vector3f> match_xyt;
            while(i != e){
                match_xyt.push_back(xyScaledTFromMat(i->second.cloud->globalTransform() * i->second.mat));
                i++;
            }
            
            i = b;
            for(unsigned j = 0; j < match_xyt.size(); j++, i++){
                // consensus with self is given
                consensus_set.clear();
                consensus_set.push_back(i);
                Eigen::Vector3f const& xyt = match_xyt[j];
                m = b;
                for(unsigned k = 0; k < match_xyt.size(); k++, m++){
                    if(k == j)
                        continue;
                    const float sq_norm = (xyt - match_xyt[k]).squaredNorm();
                    if(sq_norm < sq_tolerance)
                        consensus_set.push_back(m);
                    //debug() << "consensus? sq norm:" << sq_norm << "tolerance:" << sq_tolerance;
                }
                // include equal because higher scored matches are at the end,
                // and in the case of a consensus tie it makes sense to use
                // score to arbitrate
                if(consensus_set.size() >= best_consensus_set.size()){
                    r = i;
                    best_consensus_set = consensus_set;
                }
            }
            if(best_consensus_set.size() > m_params.min_scan_consensus ||
               best_consensus_set.size() >= m_key_scans.size() * m_params.min_scan_consensus / m_params.max_considered_overlaps)
                return r;
            return e;
        }
        
        
        /* Re-build the key-scan location cloud after an optimisation update.
         */
        void _updateKeyScanLocations(){
            m_key_scan_locations->clear();
            
            Eigen::Vector3f xyr;
            PointT xyr_space_loc;

            foreach(cloud_ptr k, m_key_scans){
                xyr = xyScaledTFromMat(k->globalTransform());
                xyr_space_loc = PointT(xyr[0], xyr[1], xyr[2]);
                m_key_scan_locations->push_back(PointT(xyr_space_loc));
            }
        }

        /* Return all cloud parts in the map that overlap with p by more than
         * m_params.overlap_threshold. The search is limited to the nearest k
         * key-scans to p.
         *
         * !!! should really use a sorted datastructure based on the bounding
         * boxes of the key scans for this search, rather than just the
         * origins. But this gives sufficiently good algorithmic complexity.
         */
        cloud_overlap_map overlappingClouds(cloud_ptr p, unsigned k = 25) const{
            std::vector<int> k_indices(k);
            std::vector<float> k_sqr_distances(k);
            const Eigen::Vector3f xyr = xyScaledTFromMat(p->globalTransform());
            const PointT xyr_space_loc = PointT(xyr[0], xyr[1], xyr[2]);

            const int n = m_key_scan_locations->nearestKSearch(xyr_space_loc, k, k_indices, k_sqr_distances);

            cloud_overlap_map r;
            for(int i = 0; i < n; i++){
                cloud_ptr m = m_key_scans[k_indices[i]];
                float overlap = overlapPercent(m, p);
                if(overlap > m_params.overlap_threshold)
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
         * overlap with existing parts is less than m_params.overlap_threshold
         */
        float overlapPercent(cloud_ptr a, cloud_ptr b) const{
            assert(a->size() != 0 && b->size() != 0);

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
                const double intersect_area = std::fabs(ClipperLib::Area(solution[0]));
                const double a_area = std::fabs(ClipperLib::Area(clipper_poly_a));
                const double b_area = std::fabs(ClipperLib::Area(clipper_poly_b));

                /*debug(3) << "overlap pct =" << 1e-6*intersect_area
                         << "/ min(" << -1e-6*a_area << "," << -1e-6*b_area << ") ="
                         << intersect_area / std::min(a_area,b_area);*/
                return intersect_area / std::min(a_area,b_area);
            }else{
                return 0;
            }
        }
        
        // return area of convex hull in m^2
        static float area(cloud_ptr a){
           assert(a->size() != 0);

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
            const bool enough_points = (p->size() > m_params.min_initial_points);
            const bool large_enough_area = enough_points && (area(p) > m_params.min_initial_area);
            if(!enough_points)
                debug() << "too few points for initialisation:" << p->size() << "<" << m_params.min_initial_points;
            else if(!large_enough_area)
                debug() << "too small area for initialisation:" << area(p) << "<" << m_params.min_initial_area;
            return enough_points && large_enough_area;
        }

        /* Convert 4d affine transformation of a scan origin into 3D (x, y,
         * scaled rotation) space in which a constant distance metric is used
         * to decide on placement of new key scans
         */
        Eigen::Vector3f xyScaledTFromMat(Eigen::Matrix4f const& a) const{
            const Eigen::Vector3f t = xyThetaFrom4DAffine(a);
            return Eigen::Vector3f(t[0], t[1], m_params.rotation_scale * t[2]);
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
                    it->second.cloud, p, it->second.mat
                ));
                it++;
            }
            
            m_key_constraints.insert(m_key_constraints.end(), r.begin(), r.end());
            return r;
        }

        
        /* dump the pose history to file */
        void dumpPoseHistory(){
            const static int pid = getpid();
            const static std::string dir = mkStr() << "/tmp/cauv/slamdump/" << pid << "/";
            static int32_t dump_num = 0;
            dump_num++;
            
            debug() << "dump pose locations to" << dir;
            std::system(std::string(mkStr() << "mkdir -p " << dir).c_str());

            std::ofstream keyfile(std::string(mkStr() << dir << "keyframes-" << dump_num << ".txt").c_str());
            int id = 0;
            foreach(cloud_ptr k, m_key_scans){
                const Eigen::Matrix4f m = k->globalTransform();
                keyfile << "keyframe " << id++ << " " << k << " "
                        << xyThetaFrom4DAffine(m).transpose() << " "
                        << k->time() << " (\n" << m << ")\n";
            }
            keyfile.close();
            
            id = 0;
            std::ofstream locfile(std::string(mkStr() << dir << "poses-" << dump_num << ".txt").c_str());
            foreach(location_ptr p, m_all_scans){
                const Eigen::Matrix4f m = p->globalTransform();
                locfile << "pose " << id++ << " " << p->relativeTo() << " "
                        << xyThetaFrom4DAffine(m).transpose() << " "
                        << p->time() << " (\n" << m << ")\n";
            }
            locfile.close();
            debug() << "pose dump complete";
        }


        // - private data
        CloudGraphParams m_params;
        
        // +1 each time graph optimiser is run - i.e. this number changes
        // whenever everything might have moved
        int m_graph_optimisation_count;
        
        // if true, write pose history to files each time the pose graph is
        // optimised
        bool m_dump_pose_history;
        
        cloud_vec m_key_scans;
        std::map<location_ptr, std::size_t> m_key_scan_indicies;

        // each point in this cloud is a key scan (same order as m_key_scans):
        // x,y,z map to x,y,m_rotation_scale*rotation 
        boost::shared_ptr< KDTreeCachingCloud<PointT> > m_key_scan_locations;
        
        
        // similarly, this will need some thought to scale well
        constraint_vec m_key_constraints;

        // for these scans, all data apart from the time and relative location
        // is discarded
        location_vec m_all_scans;
        
        // statistics about how well the classifier performed:
        unsigned m_n_passed_good;
        unsigned m_n_passed_bad;
        unsigned m_n_failed_good;
        unsigned m_n_failed_bad;
};

} // namespace cauv
} // namespace imgproc

#endif //ndef __CAUV_SONAR_SLAM_CLOUD_H__
