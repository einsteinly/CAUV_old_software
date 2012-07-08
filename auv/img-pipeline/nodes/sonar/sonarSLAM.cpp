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

#include "sonarSLAMNode.h"

#include <algorithm>
#include <limits>

#include <boost/tuple/tuple.hpp> // for tie()
#include <boost/algorithm/string.hpp> // for iequals

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
/*#ifdef CAUV_CLOUD_DUMP
#include <pcl/io/pcd_io.h>
#endif
*/

#include <opencv2/imgproc/imgproc.hpp>

#include <utility/bash_cout.h>

#include <generated/types/LocationMessage.h>

#include "mapping/slamCloud.h" 
#include "mapping/scanMatching.h"
#include "mapping/graphOptimiser.h"
#include "mapping/stuff.h"

#include "../../nodeFactory.h"

namespace cauv{
namespace imgproc{
// register this node type in the factory thing that creates nodes
DEFINE_NFR(SonarSLAMNode, NodeType::SonarSLAM);

} // namespace imgproc
} // namespace cauv

using namespace cauv;
using namespace cauv::imgproc;

typedef pcl::PointXYZ pt_t;
typedef SlamCloudPart<pt_t> cloud_t;
// actually a boost shared_ptr:
typedef cloud_t::Ptr cloud_ptr;

//static uint32_t Min_Initial_Points = 10;

// avoid including PCL stuff in the header:
namespace cauv{
namespace imgproc{

// - static functions
static void drawCircle(cv::Mat& image,
                       Eigen::Vector2f const& image_coords,
                       float scale,
                       cv::Scalar const& col){
    // !!! FIXME in my version of opencv shift doesn't work for drawing
    // circles...
    const int shift = 0;
    // NB: subpixel precision
    cv::Point centre(image_coords[0]*(1<<shift),
                     image_coords[1]*(1<<shift));
    int radius = scale * (1<<shift);
    if(radius < 1)
        radius = 1;
    cv::circle(image, centre, radius, col, 1/*thickness*/, 4/*4-connected*/, shift);
}

static void drawLine(cv::Mat& image,
                     Eigen::Vector2f const& pt1,
                     Eigen::Vector2f const& pt2,
                     cv::Scalar const& col,
                     int thickness=1){
    const int shift = 0;
    cv::Point a(pt1[0]*(1<<shift), pt1[1]*(1<<shift));
    cv::Point b(pt2[0]*(1<<shift), pt2[1]*(1<<shift));
    line(image, a, b, col, thickness, CV_AA, shift);
}

static void drawPoly(cv::Mat& image,
                     std::vector<Eigen::Vector2f> const& poly,
                     cv::Scalar const& col,
                     int thickness=1){
    const int shift = 0;
    std::vector<cv::Point> cv_pts;
    cv_pts.reserve(poly.size());
    foreach(Eigen::Vector2f const& p, poly)
        cv_pts.push_back(cv::Point(p[0]*(1<<shift), p[1]*(1<<shift)));
    const int npts = cv_pts.size();
    cv::Point const* pts = &(cv_pts[0]);
    cv::polylines(image, &pts, &npts, 1, false, col, thickness, CV_AA, shift);
}

// - local classes
class SonarSLAMImpl{
    public:
        SonarSLAMImpl()
            : m_graph(),
              m_cumulative_rotation_guess(0),
              m_vis_metres_per_px(0),
              m_vis_origin(0,0),
              m_vis_res(0,0),
              m_vis_parameters_changed(false),
              m_vis_buffer(cv::Size(0,0), CV_8UC3, cv::Scalar(0)),
              m_vis_last_cloud(cv::Size(0,0), CV_8UC3, cv::Scalar(0)),
              m_vis_keyframes_included(0),
              m_vis_allframes_included(0),
              m_vis_graphopt_number(0),
              m_async_data_mux(),
              m_have_coordinate_origin(false),
              m_coordinate_origin(WGS84Coord::fromDatum("NURC")),
              m_orientation_origin(floatYPR(0,0,0)),
              m_have_telem(false),
              m_last_orientation(){
        }

        void reset(){
            m_graph.reset();
            m_cumulative_rotation_guess = 0;
            initVis();
        }

        void onTelemetry(floatYPR const& ori, float const& depth){
            boost::unique_lock<boost::mutex> l(m_async_data_mux);
            m_have_telem = true;
            m_last_orientation = ori;
            m_last_depth = depth;
       }

        void setGraphProperties(CloudGraphParams params){
            m_graph.setParams(params);
        }

        float registerScan(cloud_ptr scan,
                           PairwiseMatcher<pt_t> const& scan_matcher,
                           GraphOptimiser const& graph_optimiser,
                           float rotation_guess_relative_to_last_scan, // radians
                           Eigen::Matrix4f& guessed_transformation,
                           Eigen::Matrix4f& global_transformation){

            m_cumulative_rotation_guess += rotation_guess_relative_to_last_scan;

            Eigen::Matrix4f rotation_guess = Eigen::Matrix4f::Identity();
            
            if(m_have_telem){
                boost::unique_lock<boost::mutex> l(m_async_data_mux);
                const Eigen::Matrix4f last_transform = m_graph.lastTransform();
                const float yaw_rad = m_last_orientation.yaw * M_PI / 180;
                Eigen::Matrix4f telemetry_rotation = Eigen::Matrix4f::Identity();
                telemetry_rotation.block<3,3>(0,0) = Eigen::Matrix3f(
                    Eigen::AngleAxisf(yaw_rad, Eigen::Vector3f::UnitZ())
                );
                rotation_guess = last_transform.inverse() * telemetry_rotation;
                debug() << BashColour::Purple << "cumulative rotation:" << m_cumulative_rotation_guess
                        << "telemetry rotation:" << yaw_rad << "(radians)";
            }else{
                rotation_guess.block<3,3>(0,0) = Eigen::Matrix3f(
                    Eigen::AngleAxisf(m_cumulative_rotation_guess, Eigen::Vector3f::UnitZ())
                );
            }


            Eigen::Matrix4f guess = m_graph.guessTransformationAtTime(
                scan->time(), rotation_guess
            ); 
            
            guessed_transformation = guess;

            const float r = m_graph.registerScan(
                scan, guess, scan_matcher, graph_optimiser, global_transformation
            );

            if(r != 0){
                // reset cumulative rotation since last scan matched
                m_cumulative_rotation_guess = 0;
            }else{
                debug() << "cumulative rotation guess grows to:" << m_cumulative_rotation_guess;
            }

            return r;
        }

        void setVisProperties(Eigen::Vector2i const& res,
                              Eigen::Vector2f const& origin,
                              float const& m_per_px){
            if(m_vis_metres_per_px != m_per_px ||
               m_vis_origin != origin ||
               m_vis_res != res){
                m_vis_metres_per_px = m_per_px;
                m_vis_origin = origin;
                m_vis_res = res;
                m_vis_parameters_changed = true;
            }
        }

        void initVis(){
            m_vis_parameters_changed = false;
            m_vis_buffer = cv::Mat(cv::Size(m_vis_res[0], m_vis_res[1]), CV_8UC3, cv::Scalar(0));
            m_vis_last_cloud = cv::Mat(cv::Size(m_vis_res[0], m_vis_res[1]), CV_8UC3, cv::Scalar(0));
            m_vis_keyframes_included = 0;
            m_vis_allframes_included = 0;
            m_vis_graphopt_number = m_graph.graphOptimisationsCount();
        }
        
        // reverse the y-axis (back to OpenCV conventional image coordinates,
        // subtract the visualisation origin, and scale
        Eigen::Vector2f toVisCoords(Eigen::Vector3f const& p) const{
            return (Eigen::Vector2f(p[0],-p[1]) - m_vis_origin) / m_vis_metres_per_px;
        }

        void updateVis(){
            if(m_vis_parameters_changed || m_vis_graphopt_number != m_graph.graphOptimisationsCount())
                initVis();

            typedef SlamCloudGraph<pt_t>::cloud_vec cloud_vec;

            cloud_vec::const_iterator i;
            cloud_vec const& key_scans = m_graph.keyScans();
            for(i = key_scans.begin() + m_vis_keyframes_included; i != key_scans.end(); i++){
                Eigen::Matrix4f const& global_transform = (*i)->globalTransform();
                const Eigen::Vector2f image_pt = toVisCoords(
                    global_transform.block<3,1>(0,3)
                );
                drawCircle(
                    m_vis_buffer, image_pt, 0.5/m_vis_metres_per_px,
                    cv::Scalar(80, 255, 60)
                );
                //const Eigen::Vector2f based_on_image_pt = toVisCoords(
                //    (*i)->relativeTo()->globalTransform().block<3,1>(0,3)
                //);
                //drawLine(
                //    m_vis_buffer, image_pt, based_on_image_pt,
                //    cv::Scalar(60, 235, 40)
                //);
                
                // draw relative constraints:
                foreach(location_ptr p, (*i)->constrainedTo()){
                    const Eigen::Vector2f to_img_pt = toVisCoords(
                        p->globalTransform().block<3,1>(0,3)
                    );
                    drawLine(
                        m_vis_buffer, image_pt, to_img_pt,
                        cv::Scalar(40, 170, 30)
                    );
                }
                // draw all the constraints completely relaxed: (ie, they won't
                // end in nodes)
                foreach(Eigen::Vector3f const& p, (*i)->constraintEndsGlobal()){
                    const Eigen::Vector2f to_img_pt = toVisCoords(p);
                    drawLine(
                        m_vis_buffer, image_pt, to_img_pt,
                        cv::Scalar(20, 110, 110)
                    );
                    drawCircle(
                        m_vis_buffer, to_img_pt, 0.25/m_vis_metres_per_px,
                        cv::Scalar(20, 110, 110)
                    );
                }

                for(std::size_t j = 0; j < (*i)->size(); j++){
                    const Eigen::Vector2f pt = toVisCoords(
                        (*i)->globalTransform().block<3,3>(0,0) * (**i)[j].getVector3fMap() +
                        (*i)->globalTransform().block<3,1>(0,3)
                    );
                    drawCircle(
                        m_vis_buffer, pt, 0.1/m_vis_metres_per_px,
                        cv::Scalar(160, 160, 160)
                    );
                }

                cloud_t::base_cloud_ptr hull_cloud;
                std::vector<pcl::Vertices> hull_polys;
                (*i)->getGlobalConvexHull(hull_cloud, hull_polys);
                assert(hull_polys.size() == 1);

                std::vector<Eigen::Vector2f> image_hull_pts;
                image_hull_pts.reserve(hull_polys[0].vertices.size());
                for(std::size_t j = 0; j < hull_polys[0].vertices.size(); j++){
                    image_hull_pts.push_back(toVisCoords(
                        (*hull_cloud)[hull_polys[0].vertices[j]].getVector3fMap()
                    ));
                }
                //drawPoly(
                //    m_vis_buffer, image_hull_pts, cv::Scalar(100,90,100)
                //);
                /*foreach(Eigen::Vector2f const& p, image_hull_pts)
                    drawCircle(
                        m_vis_buffer, p, 0.1/m_vis_metres_per_px, cv::Scalar(0,0,140)
                    );*/
            }
            m_vis_keyframes_included = key_scans.size();

            location_vec::const_iterator j;
            location_vec const& all_scans = m_graph.allScans();

            for(j = all_scans.begin() + m_vis_allframes_included; j != all_scans.end(); j++){
                Eigen::Matrix4f const& global_transform = (*j)->globalTransform();
                const Eigen::Vector2f image_pt = toVisCoords(
                    global_transform.block<3,1>(0,3)
                );
                drawCircle(
                    m_vis_buffer, image_pt, 0.25/m_vis_metres_per_px,
                    cv::Scalar(105, 40, 40)
                );
                if((*j)->relativeTo()){
                    const Eigen::Vector2f based_on_image_pt = toVisCoords(
                        (*j)->relativeTo()->globalTransform().block<3,1>(0,3)
                    );
                    /*drawLine(
                        m_vis_buffer, image_pt, based_on_image_pt,
                        cv::Scalar(100, 120, 50)
                    );*/
                }
            }
            m_vis_allframes_included = all_scans.size();
        }

        void updateLastScanVis(cloud_ptr p, cv::Scalar const& colour, bool clear=false){
            if(clear)
                m_vis_last_cloud = cv::Mat(cv::Size(m_vis_res[0], m_vis_res[1]), CV_8UC3, cv::Scalar(0));

            Eigen::Matrix4f const& global_transform = p->globalTransform();
            const Eigen::Vector2f image_pt = toVisCoords(
                global_transform.block<3,1>(0,3)
            );

            for(std::size_t j = 0; j < p->size(); j++){
                const Eigen::Vector2f pt = toVisCoords(
                    p->globalTransform().block<3,3>(0,0) * (*p)[j].getVector3fMap() +
                    p->globalTransform().block<3,1>(0,3)
                );
                drawCircle(
                    m_vis_last_cloud, pt, 0.1/m_vis_metres_per_px,
                    colour
                );
            }
        }

        cv::Mat const& vis() const{
            return m_vis_buffer;
        }

        cv::Mat const& lastScanVis() const{
            return m_vis_last_cloud;
        }

        WGS84Coord const& wgs84AtOrigin(){
            boost::unique_lock<boost::mutex> l(m_async_data_mux);
            return m_coordinate_origin;
        }

        void onGPSLoc(WGS84Coord const& c){
            boost::unique_lock<boost::mutex> l(m_async_data_mux);
            m_coordinate_origin = c;
            if(!m_have_coordinate_origin && m_have_telem){
                m_have_coordinate_origin = true;
                m_coordinate_origin = c;
            }
        }
        
        // get a location message representing the current state.
        // depth is derived 
        boost::shared_ptr<LocationMessage> locationMessage(){
            boost::unique_lock<boost::mutex> l(m_async_data_mux);
            NorthEastDepth position = m_graph.position(m_orientation_origin.yaw * M_PI / 180, m_last_depth);
            WGS84Coord coord = m_coordinate_origin + position;
            return boost::make_shared<LocationMessage>(coord, m_graph.speed());
        }

    private:
        SlamCloudGraph<pt_t> m_graph;
        
        float m_cumulative_rotation_guess;

        float m_vis_metres_per_px;
        Eigen::Vector2f m_vis_origin;
        Eigen::Vector2i m_vis_res;
        bool m_vis_parameters_changed;

        cv::Mat m_vis_buffer;
        cv::Mat m_vis_last_cloud;
        int m_vis_keyframes_included;
        int m_vis_allframes_included;
        int m_vis_graphopt_number;

        boost::mutex m_async_data_mux;
        
        bool m_have_coordinate_origin;
        WGS84Coord m_coordinate_origin;
        floatYPR m_orientation_origin;
        
        bool m_have_telem;
        floatYPR m_last_orientation;
        float m_last_depth;
        
};

} // namespace imgproc
} // namespace cauv


// - SonarSLAMNode implementation
void SonarSLAMNode::init(){
    typedef std::vector<KeyPoint> kp_vec;

    m_impl = boost::make_shared<SonarSLAMImpl>();

    // slow node (don't schedule nodes providing input until we've
    // finished doing work here)
    //m_speed = slow; // !!! slow can cause inputs to get out of sync under high load
    m_speed = fast;

    // inputs:
    registerParamID("keypoints", kp_vec(), "(xy) keypoints used to update map", Must_Be_New);
    registerParamID("training: polar keypoints", kp_vec(), "", Must_Be_New);
    requireSyncInputs("keypoints", "training: polar keypoints");
    
    registerInputID("keypoints image", Optional); // image from which the keypoints came: actually just passed straight back out with the keypoints training data
    registerParamID("delta theta", float(0), "estimated change in orientation (radians) since last image", Must_Be_New);

    // parameters: may be old
    // Parameters that are really inputs:
    registerInputID("xy image", /*"image to use for map visualisation (may be unconnected)", */Optional);
    registerParamID("xy metres/px", float(1), "range conversion for visualisation image");

    // Control Parameters:
    registerParamID("clear", bool(false), "true => discard accumulated point cloud");
    ///*unused*/ registerParamID("map merge alpha", float(5), "alpha-hull parameter for map merging");
    registerParamID("score threshold", float(2), "keypoint set will be rejected if mean distance error is greater than this");
    registerParamID("weight test", float(5), "keypoints with weights greater than this will be used for registration");
    registerParamID("feature merge distance", float(0.05), "keypoints closer to each other than this will be merged"); // now used for deciding whether points were good for training
    registerParamID("overlap threshold", float(0.3), "overlap of convex hulls required to consider matching a pair of scans");
    registerParamID("keyframe spacing", float(2.0), "minimum distance between keyframes");

    // Registration Parameters:
    registerParamID("max iters", int(20), "");
    registerParamID("transform eps", float(1e-9), "difference between transforms in successive iters for convergence");
    registerParamID("euclidean fitness", float(1e-7), "max error between consecutive steps for non-convergence");
    // ICP only:
    registerParamID("reject threshold", float(0.5), "RANSAC outlier rejection distance");
    registerParamID("max correspond dist", float(1), ""); // also used for fitness score of NDT
    registerParamID("ransac iterations", int(10), "RANSAC iterations");

    // NDT only:
    registerParamID("grid step", float(2.5), "NDT grid step");

    // ICP / NDT / NDT non-linear
    registerParamID("match algorithm", std::string("ICP"), "ICP, Non-Linear ICP, or NDT");
    
    // Graph Optimisation Parameters
    registerParamID("graph iters", int(10), "Iterations of graph optimisation per key-scan");
    registerParamID("max matches", int(3), "Maximum number of pairwise correspondences for each scan");
    registerParamID("match consensus", int(2), "Consensus required for match");
    registerParamID("consensus tolerance", float(0.3), "Metres between matches to consider consensus");

    // Visualisation Parameters:
    //registerParamID("-render resolution", float(400), "in pixels");
    //registerParamID("-render size", float(100), "in metres");
    //registerParamID("-render origin x", int(400));
    //registerParamID("-render origin y", int(400));

    registerParamID("-vis size", float(100));
    registerParamID("-vis resolution", int(600));
    registerParamID("-vis origin x", float(0));
    registerParamID("-vis origin y", float(0));

    // outputs:
    registerOutputID("cloud visualisation");
    //registerOutputID("render");

    // training outputs for feature extractor:
    registerOutputID("training: keypoints", kp_vec());
    registerOutputID("training: keypoints image");
    registerOutputID("training: goodness", std::vector<int>());
}

void SonarSLAMNode::onGPSLoc(boost::shared_ptr<GPSLocationMessage const> m){
    debug() << "SLAM GPS loc:" << m;
    m_impl->onGPSLoc(m->location());
}

void SonarSLAMNode::onTelemetry(boost::shared_ptr<TelemetryMessage const> m){
    debug() << "SLAM Telemetry:" << m->orientation();
    m_impl->onTelemetry(m->orientation(), m->depth());
}

void SonarSLAMNode::doWork(in_image_map_t& inputs, out_map_t& r){
    typedef std::vector<KeyPoint> kp_vec;

    bool clear = param<bool>("clear");
    if(clear)
        m_impl->reset();
    
    // Synchronised parameters: keypoints & training_keypoints
    kp_vec keypoints;
    UID keypoints_uid;
    boost::tie(keypoints, keypoints_uid) = paramAndUID<kp_vec>("keypoints");

    // Because we set the parameters as synchronised, there's guaranteed to be
    // training keypoints with the same UID
    const kp_vec training_keypoints = param<kp_vec>("training: polar keypoints", keypoints_uid);
    
    // or, should be guaranteed... (cases where it's been broken: swapping
    // between two different keypoints extractors based on the same source --
    // so UIDs match, but keypoints do not correspond)
    if(keypoints.size() != training_keypoints.size())
        throw std::runtime_error("training keypoints do not correspond to keypoints");
    
    // The rest of the parameters
    const int   max_iters   = param<int>("max iters");
    const float euclidean_fitness   = param<float>("euclidean fitness");
    const float transform_eps       = param<float>("transform eps");
    const float reject_threshold    = param<float>("reject threshold");
    const float max_correspond_dist = param<float>("max correspond dist");
    const std::string match_algorithm = param<std::string>("match algorithm");    
    const int ransac_iters          = param<int>("ransac iterations");
    const float grid_step           = param<float>("grid step");

    const int graph_iters = param<int>("graph iters");
    const int max_matches = param<int>("max matches");
    const int require_match_consensus = param<int>("match consensus");
    const float consensus_tolerance = param<float>("consensus tolerance");

    const float score_thr   = param<float>("score threshold");
    const float delta_theta = param<float>("delta theta");
    const float weight_test = param<float>("weight test");
    const float point_merge_distance = param<float>("feature merge distance");
    //const float map_merge_alpha = param<float>("map merge alpha");
    const float overlap_threshold = param<float>("overlap threshold");
    const float keyframe_spacing = param<float>("keyframe spacing");

    const Eigen::Vector2f vis_origin(param<float>("-vis origin x"),
                                     param<float>("-vis origin y"));
    const Eigen::Vector2i vis_res(param<int>("-vis resolution"),
                                  param<int>("-vis resolution"));
    const float vis_size = param<float>("-vis size");
    image_ptr_t keypoints_image = inputs["keypoints image"];

    //const float xy_m_per_pix = param<float>("xy metres/px");
    //const int render_size = param<int>("-vis size");
    //const Eigen::Vector2f render_origin(param<int>("-render origin x"),
    //                                 param<int>("-render origin y"));

    CloudGraphParams params;
    params.overlap_threshold = overlap_threshold;
    params.keyframe_spacing = keyframe_spacing;
    //params.min_initial_points = 10;
    //params.min_initial_area = 5;
    params.good_keypoint_distance = point_merge_distance;
    //params.max_speed = 2.0;
    params.max_considered_overlaps = max_matches;
    params.min_scan_consensus = require_match_consensus;
    params.scan_consensus_tolerance = consensus_tolerance;
    //params.rotation_scale = 4;

    m_impl->setGraphProperties(params);
    m_impl->setVisProperties(vis_res, vis_origin, vis_size/vis_res[0]);

    image_ptr_t xy_image = inputs["xy image"];

    TimeStamp ts;
    if(keypoints_image)
        ts = keypoints_image->ts();
    else
        ts = now();
    
    cloud_ptr scan = boost::make_shared<cloud_t>(
        keypoints, ts, cloud_t::FilterResponse(weight_test)
    );
    
    boost::shared_ptr< PairwiseMatcher<pt_t> > scan_matcher;
    if(boost::iequals(match_algorithm, "NDT")){
        scan_matcher = makeNDTPairwiseMatcherShared(
            max_iters, euclidean_fitness, transform_eps, max_correspond_dist, score_thr, grid_step
        );
    }else if(boost::iequals(match_algorithm, "ICP")){
        scan_matcher = makeICPPairwiseMatcherShared(
            max_iters, euclidean_fitness, transform_eps,
            reject_threshold, max_correspond_dist, score_thr, ransac_iters
        );
    }else if(boost::iequals(match_algorithm, "non-linear ICP")){
        scan_matcher = makeICPNonLinearPairwiseMatcherShared(
            max_iters, euclidean_fitness, transform_eps,
            reject_threshold, max_correspond_dist, score_thr, ransac_iters
        );
    }else{
        throw parameter_error(
            "invalid scan matching method: \""+ match_algorithm +
            "\": valid methods are ICP, non-linear ICP, NDT"
        );
    }

    GraphOptimiserV1 graph_optimiser(graph_iters);
    
    debug() << "external delta theta =" << delta_theta * 180 / 3.14159 << "degrees";
    /*Eigen::Matrix4f relative_rotation_guess = Eigen::Matrix4f::Identity();
    relative_rotation_guess.block<3,3>(0,0) = Eigen::Matrix3f(
        Eigen::AngleAxisf(delta_theta, Eigen::Vector3f::UnitZ())
    );*/

    Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Zero();
    Eigen::Matrix4f guessed_transformation = Eigen::Matrix4f::Zero();

    const float confidence = m_impl->registerScan(
        scan,
        *scan_matcher,
        graph_optimiser,
        //relative_rotation_guess,
        // TODO: !!! the sign should be sorted out before this point
        -delta_theta,
        guessed_transformation,
        global_transformation
    );

    //info() << "sonarSLAM scan confidence" << confidence;

    m_impl->updateVis();

    sendMessage(m_impl->locationMessage());
    
    #ifdef CAUV_CLOUD_VISUALISATION
    // potentially expensive: visualise every cloud:
    if(confidence > 0){
        // visualise at final position
        m_impl->updateLastScanVis(scan, cv::Scalar(80, 80, 80), true);
    }else{
        // visualise at guess and final position:
        scan->setRelativeToNone();
        scan->setRelativeTransform(guessed_transformation);
        m_impl->updateLastScanVis(scan, cv::Scalar(130, 10, 20), true);
        scan->setRelativeTransform(global_transformation);
        m_impl->updateLastScanVis(scan, cv::Scalar(10, 10, 120));
    }
    #endif

    if(m_impl->vis().rows > 0 && m_impl->vis().cols > 0){
        cv::Mat vis = m_impl->vis();//.clone();
        if(m_impl->lastScanVis().rows == m_impl->vis().rows &&
           m_impl->lastScanVis().cols == m_impl->vis().cols)
            vis += m_impl->lastScanVis();
        r["cloud visualisation"] = boost::make_shared<Image>(vis);
    }

    r["training: keypoints"] = training_keypoints;
    r["training: keypoints image"] = inputs["keypoints image"];
    r["training: goodness"] = scan->keyPointGoodness();
}

