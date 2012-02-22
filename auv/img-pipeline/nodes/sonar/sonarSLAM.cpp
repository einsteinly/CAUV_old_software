/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
/*#ifdef CAUV_CLOUD_DUMP
#include <pcl/io/pcd_io.h>
#endif
*/

#include <opencv2/imgproc/imgproc.hpp>

#include <utility/bash_cout.h>

#include "mapping/slamCloud.h"

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

/*
static Eigen::Vector3f xythetaFrom4dAffine(Eigen::Matrix4f const& transform){
    // split transform into affine parts:
    const Eigen::Matrix3f rotate    = transform.block<3,3>(0, 0);
    const Eigen::Vector3f translate = transform.block<3,1>(0, 3);

    const Eigen::Vector3f t = rotate*Eigen::Vector3f(1,0,0);
    const float rz = (180/M_PI)*std::atan2(t[1], t[0]);
    return Eigen::Vector3f(translate[0], translate[1], rz);
}
*/

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
              m_vis_metres_per_px(0),
              m_vis_origin(0,0),
              m_vis_res(0,0),
              m_vis_parameters_changed(false),
              m_vis_buffer(cv::Size(0,0), CV_8UC3, cv::Scalar(0)),
              m_vis_keyframes_included(0),
              m_vis_allframes_included(0){
        }

        void reset(){
            m_graph.reset();
            initVis();
        }

        void setGraphProperties(float overlap_threshold,
                                float keyframe_spacing,
                                float min_initial_points,
                                float good_keypoint_distance){
            m_graph.setParams(
                overlap_threshold, keyframe_spacing, min_initial_points,
                good_keypoint_distance
            );
        }


        float registerScan(cloud_ptr scan,
                           PairwiseMatcher<pt_t> const& scan_matcher,
                           Eigen::Matrix4f const& external_guess,
                           Eigen::Matrix4f& global_transformation){
            Eigen::Matrix4f guess = m_graph.guessTransformationAtTime(scan->time());

            // use rotation from external guess, and translation from internal
            // guess:
            if(external_guess.block<3,1>(0,3).norm() > 1e-3)
                warning() << "external translation prediction is ignored";
            guess.block<3,3>(0,0) = external_guess.block<3,3>(0,0);

            return m_graph.registerScan(scan, guess, scan_matcher, global_transformation);
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
            m_vis_buffer = cv::Mat(cv::Size(m_vis_res[0], m_vis_res[1]), CV_8UC3, cv::Scalar(0)),
            m_vis_keyframes_included = 0;
            m_vis_allframes_included = 0;
        }

        Eigen::Vector2f toVisCoords(Eigen::Vector3f const& p) const{
            return (p.block<2,1>(0,0) - m_vis_origin) / m_vis_metres_per_px;
        }

        void updateVis(){
            if(m_vis_parameters_changed)
                initVis();

            typedef SlamCloudGraph<pt_t>::cloud_vec cloud_vec;
            typedef SlamCloudGraph<pt_t>::location_vec location_vec;

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
                drawPoly(
                    m_vis_buffer, image_hull_pts, cv::Scalar(100,90,100)
                );
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
                    drawLine(
                        m_vis_buffer, image_pt, based_on_image_pt,
                        cv::Scalar(97.5, 160, 50)
                    );
                }
            }
            m_vis_allframes_included = all_scans.size();
        }

        cv::Mat const& vis() const{
            return m_vis_buffer;
        }

    private:
        SlamCloudGraph<pt_t> m_graph;

        float m_vis_metres_per_px;
        Eigen::Vector2f m_vis_origin;
        Eigen::Vector2i m_vis_res;
        bool m_vis_parameters_changed;

        cv::Mat m_vis_buffer;
        int m_vis_keyframes_included;
        int m_vis_allframes_included;

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
    // ... not implemented yet
    //requireSyncInputs("keypoints", "training: polar keypoints");
    
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

    // ICP Parameters:
    registerParamID("max iters", int(20), "");
    registerParamID("transform eps", float(1e-9), "difference between transforms in successive iters for convergence");
    registerParamID("euclidean fitness", float(1e-7), "max error between consecutive steps for non-convergence");
    registerParamID("reject threshold", float(0.5), "RANSAC outlier rejection distance");
    registerParamID("max correspond dist", float(1), "");

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

void SonarSLAMNode::doWork(in_image_map_t& inputs, out_map_t& r){
    typedef std::vector<KeyPoint> kp_vec;

    bool clear = param<bool>("clear");
    if(clear)
        m_impl->reset();

    const std::vector<KeyPoint> keypoints = param<kp_vec>("keypoints");
    const std::vector<KeyPoint> training_keypoints = param<kp_vec>("training: polar keypoints");
    assert(keypoints.size() == training_keypoints.size());

    const int   max_iters   = param<int>("max iters");
    const float euclidean_fitness   = param<float>("euclidean fitness");
    const float transform_eps       = param<float>("transform eps");
    const float reject_threshold    = param<float>("reject threshold");
    const float max_correspond_dist = param<float>("max correspond dist");

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

    //const float xy_m_per_pix = param<float>("xy metres/px");
    //const int render_size = param<int>("-vis size");
    //const Eigen::Vector2f render_origin(param<int>("-render origin x"),
    //                                 param<int>("-render origin y"));
    
    m_impl->setGraphProperties(overlap_threshold, keyframe_spacing, 10, point_merge_distance);
    m_impl->setVisProperties(vis_res, vis_origin, vis_size/vis_res[0]);

    image_ptr_t xy_image = inputs["xy image"];


    // !!! TODO: propagate timestamps with sonar images, or something
    cloud_ptr scan = boost::make_shared<cloud_t>(
        keypoints, now(), SlamCloudPart<pt_t>::FilterResponse(weight_test)
    );

    ICPPairwiseMatcher<pt_t> scan_matcher(
        max_iters, euclidean_fitness, transform_eps,
        reject_threshold, max_correspond_dist, score_thr
    );

    Eigen::Matrix4f relative_transformation_guess = Eigen::Matrix4f::Identity();
    relative_transformation_guess.block<3,3>(0,0) = Eigen::Matrix3f(
        Eigen::AngleAxisf(delta_theta, Eigen::Vector3f::UnitZ())
    );

    Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Zero();

    const float confidence = m_impl->registerScan(
        scan,
        scan_matcher,
        relative_transformation_guess,
        global_transformation
    );

    info() << "sonarSLAM scan confidence" << confidence;

    m_impl->updateVis();

    if(m_impl->vis().rows > 0 && m_impl->vis().cols > 0)
        r["cloud visualisation"] = boost::make_shared<Image>(m_impl->vis());

    r["training: keypoints"] = training_keypoints;
    r["training: keypoints image"] = inputs["keypoints image"];
    r["training: goodness"] = scan->keyPointGoodness();


#if 0
    Eigen::Matrix4f delta_transformation = Eigen::Matrix4f::Identity();
    delta_transformation.block<3,3>(0,0) = Eigen::Matrix3f(Eigen::AngleAxisf(delta_theta, Eigen::Vector3f::UnitZ()));

    Eigen::Matrix4f guess = m_impl->last_transformation * delta_transformation;
    // squish the z translation
    guess(2,3) = 0;
    //guess(0,2) = 0; guess(1,2) = 0;
    //guess(2,0) = 0; guess(2,1) = 0;


    // weight-test throwing away low scorers:
    cloud_ptr new_cloud = kpsToCloud(
        keypoints,
        guess,
        weight_test
    );

    if(!m_impl->whole_cloud){
        if(new_cloud->size() > Min_Initial_Points){
            info() << BashColour::Green
                   << "new cloud with" << new_cloud->size() << "points";
            m_impl->init(new_cloud);
            debug(3) << "transformation is:\n" << m_impl->last_transformation;

            r["whole cloud vis"] = boost::make_shared<Image>(renderCloud(
                *(m_impl->whole_cloud), Eigen::Vector2f(0,0), m_impl->min(), m_impl->max(), render_sz, false
            ));
        }else{
            r["whole cloud vis"] = boost::make_shared<Image>(cv::Mat(render_sz[0],render_sz[1],CV_8UC1,cv::Scalar(0)));
            debug() << "not enough points ("
                    << new_cloud->size() << "<" << Min_Initial_Points
                    << ") to initialise whole cloud";
        }
        r["last added vis"] = boost::make_shared<Image>(cv::Mat(render_sz[0],render_sz[1],CV_8UC1,cv::Scalar(0)));
    }else{
        debug() << "trying to match" << new_cloud->size() << "points to cloud";

        //pcl::IterativeClosestPointNonLinear<pt_t,pt_t> icp;
        pcl::IterativeClosestPoint<pt_t,pt_t> icp;
        icp.setInputCloud(new_cloud);
        icp.setInputTarget(m_impl->whole_cloud);
        //icp.setInputTarget(m_impl->last_cloud);

        icp.setMaxCorrespondenceDistance(max_correspond_dist);
        icp.setMaximumIterations(max_iters);
        icp.setTransformationEpsilon(transform_eps);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness);
        icp.setRANSACOutlierRejectionThreshold(reject_threshold);

        /* weight-test using indices vector:
        boost::shared_ptr< std::vector<int> >indices = boost::make_shared<std::vector<int> >();
        for(size_t i =0; i < new_cloud->size(); i++)
            if(new_cloud->points[i].getVector4fMap()[3] > weight_test)
                indices->push_back(i);
        debug() << indices->size() << "points pass weight test";
        icp.setIndices(indices);
        */

        // do the hard work!
        cloud_ptr final = boost::make_shared<cloud_t>();
        icp.align(*final);

        // icp doesn't copy data in the derived class...
        final->descriptors() = new_cloud->descriptors();
        final->ptIndices() = new_cloud->ptIndices();

        // guess was applied before icp determined to remaining transformation,
        // so post-multiply (ie, apply guess first)
        Eigen::Matrix4f final_transform = icp.getFinalTransformation() * guess;
        final->transformation() = final_transform;

        /*
        debug() << "guess:\n" << guess;
        debug() << "icp transform:\n" << icp.getFinalTransformation();
        debug() << "final transform:\n" << final_transform;

        // sanity check final transform:
        Eigen::Vector4f sc = final_transform * Eigen::Vector4f(0,0,0,1);
        Eigen::Vector3f torg = final->back().getVector3fMap();
        if((sc.block<3,1>(0,0) - torg).norm() > 1e-6)
            warning() << "final transformation sanity check failed:"
                      << sc[0] << "," << sc[1] << "," << sc[2] << "!="
                      << torg[0] << "," << torg[1] << "," << torg[2];
        */

        // high is bad (score is sum of squared euclidean distances)
        float score = icp.getFitnessScore();
        info() << BashColour::Green
               << "converged:" << icp.hasConverged()
               << "score:" << score;
        debug(3) << "transformation:\n" << final_transform;
        if(icp.hasConverged() && score < score_thr){
            // start out assuming all keypoints that have passed the weight
            // test are good:
            std::vector<int> point_goodness(training_keypoints.size(), 0);
            for(std::size_t i = 0; i < final->ptIndices().size(); i++)
                point_goodness[final->ptIndices()[i]] = 1;

            m_impl->addNewCloud(final, point_merge_distance, map_merge_alpha, point_goodness);

            if(xy_image){
                m_impl->updateVis(xy_image->mat(), final_transform, m_per_pix);
                r["mosaic"] = boost::make_shared<Image>(m_impl->vis());
            }

            Eigen::Vector3f xytheta = xythetaFrom4dAffine(final_transform);
            info() << BashColour::Green
                   << "added transformed points at: " << xytheta[0] << "," << xytheta[1] << "rot=" << xytheta[2] << "degress";

            r["last added vis"] = boost::make_shared<Image>(renderCloud(
                *final, Eigen::Vector2f(xytheta[0],xytheta[1]), m_impl->min(), m_impl->max(), render_sz, false
            ));

            r["training: keypoints"] = training_keypoints;
            r["training: keypoints image"] = inputs["keypoints image"];
            r["training: goodness"] = point_goodness;

        }else if(score >= score_thr){
            r["last added vis"] = boost::make_shared<Image>(cv::Mat(render_sz[0],render_sz[1],CV_8UC1,cv::Scalar(0)));
            info() << BashColour::Brown
                   << "points will not be added to the whole cloud (error too high: "
                   << score << ">=" << score_thr <<")";
        }else{
            r["last added vis"] = boost::make_shared<Image>(cv::Mat(render_sz[0],render_sz[1],CV_8UC1,cv::Scalar(0)));
            info() << BashColour::Red
                   << "points will not be added to the whole cloud (not converged)";
        }

        r["whole cloud vis"] = boost::make_shared<Image>(renderCloud(
            *(m_impl->whole_cloud), Eigen::Vector2f(0,0), m_impl->min(), m_impl->max(), render_sz, false
        ));
    }
#endif // 0
}

