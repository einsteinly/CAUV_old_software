#include "sonarSLAMNode.h"

#include <algorithm>
#include <limits>

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#ifdef CAUV_CLOUD_DUMP
#include <pcl/io/pcd_io.h>
#endif
#ifdef CAUV_CLOUD_VISUALISATION
#include <pcl/visualization/cloud_viewer.h>
#endif

#include <opencv2/imgproc/imgproc.hpp>

#include <utility/bash_cout.h>

#include "slamCloud.h"

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
typedef SlamCloud<pt_t> cloud_t;
// actually a boost shared_ptr:
typedef cloud_t::Ptr cloud_ptr;

static uint32_t Min_Initial_Points = 10;

// avoid including PCL stuff in the header:
namespace cauv{
namespace imgproc{

static Eigen::Vector3f xythetaFrom4dAffine(Eigen::Matrix4f const& transform){
    // split transform into affine parts:
    const Eigen::Matrix3f rotate    = transform.block<3,3>(0, 0);
    const Eigen::Vector3f translate = transform.block<3,1>(0, 3);

    const Eigen::Vector3f t = rotate*Eigen::Vector3f(1,0,0);
    const float rz = (180/M_PI)*std::atan2(t[1], t[0]);
    return Eigen::Vector3f(translate[0], translate[1], rz);
}

class SonarSLAMImpl{
    public:
        const static int Vis_Res = 800;

        SonarSLAMImpl()
            : whole_cloud(),
              last_cloud(),
              clouds(),
              last_transformation(Eigen::Matrix4f::Identity()),
              #ifdef CAUV_CLOUD_VISUALISATION
              viewer(boost::make_shared<pcl::visualization::CloudViewer>(mkStr() << "SonarSLAM " << ++viewer_count)),
              #endif
              m_min(std::numeric_limits<float>().max(), std::numeric_limits<float>().max()),
              m_max(-std::numeric_limits<float>().max(), -std::numeric_limits<float>().max()),
              m_vis_buffer(cv::Size(Vis_Res,Vis_Res), CV_8UC1, cv::Scalar(0)),
              m_vis_origin(Vis_Res/2,Vis_Res/2){
        }

        void init(cloud_ptr cloud){
            last_transformation = Eigen::Matrix4f::Identity();
            whole_cloud = cloud;
            last_cloud = cloud;
            updateMinMax(*cloud);
        }

        void reset(){
            whole_cloud.reset();
            clouds.clear();
            m_vis_buffer = cv::Mat(cv::Size(Vis_Res,Vis_Res), CV_8UC1, cv::Scalar(0));
        }

        void addNewCloud(cloud_ptr cloud,
                         float point_merge_distance,
                         float concave_hull_alpha){
            // TODO: propagate sonar timestamp with keypoints... somehow... and use
            // that instead of an index
            static uint32_t cloud_num = 0;
            clouds[++cloud_num] = cloud;
            //whole_cloud->mergeCollapseNearest(cloud, point_merge_distance);
            whole_cloud->mergeOutsideConcaveHull(cloud, concave_hull_alpha);
            last_transformation = cloud->transformation();            
            last_cloud = cloud;

            #ifdef CAUV_CLOUD_VISUALISATION
            viewer->showCloud(whole_cloud);
            #endif

            #ifdef CAUV_CLOUD_DUMP
            static uint32_t n = 0;
            pcl::io::savePCDFile(mkStr() << "sonarSLAM" << n++ << ".pcd", *whole_cloud);
            pcl::io::savePCDFile(mkStr() << "sonarSLAM-scan" << n++ << ".pcd", *cloud);
            pcl::io::savePCDFile(mkStr() << "sonarSLAM.pcd",  *whole_cloud);
            #endif

            updateMinMax(*cloud);
        }


        static uint8_t minMaxScreenBlend(uint8_t src, uint8_t dst){
            return std::min(std::max(src, dst), uint8_t(0xff - ((0xff-src)*(0xff-dst))/0xff));
        }
        void updateVis(cv::Mat img, Eigen::Matrix4f transformation, float m_per_px){
            cv::Mat tmp = cv::Mat(cv::Size(Vis_Res,Vis_Res), CV_8UC1, cv::Scalar(0));
            Eigen::Vector2f img_origin(img.cols/2.0, img.rows/2.0);
            Eigen::Vector3f xytheta = xythetaFrom4dAffine(transformation);
            cv::Mat tr = cv::getRotationMatrix2D(cv::Point2f(img_origin[0], img_origin[1]), -xytheta[2], 1.0);
            tr.at<double>(0,2) += xytheta[0] / m_per_px + m_vis_origin[0]-img_origin[0];
            tr.at<double>(1,2) += xytheta[1] / m_per_px + m_vis_origin[1]-img_origin[1];
            // draw a + at the origin:
            const int S = 5;
            for(int x = -S; x <= S; x++)
                for(int y = -S*(!x); y <= S*(!x); y++)
                    img.at<uint8_t>(int(0.5+img_origin[0]+x),int(0.5+img_origin[1]+y)) = 0xff;

            cv::warpAffine(img, tmp, tr, m_vis_buffer.size(), CV_INTER_LINEAR);
            for(int i = 0; i < Vis_Res; i++)
                for(int j = 0; j < Vis_Res; j++)
                    m_vis_buffer.at<uint8_t>(i,j) = minMaxScreenBlend(m_vis_buffer.at<uint8_t>(i,j), tmp.at<uint8_t>(i,j));
        }

        cv::Mat const& vis(){
            return m_vis_buffer;
        }

        void updateMinMax(cloud_t const& cloud){
            foreach(pt_t const& p, cloud.points){
                if(p.x < m_min[0]) m_min[0] = p.x;
                if(p.x > m_max[0]) m_max[0] = p.x;
                if(p.y < m_min[1]) m_min[1] = p.y;
                if(p.y > m_max[1]) m_max[1] = p.y;
            }
        }

        Eigen::Vector2f min() const{ return m_min; }
        Eigen::Vector2f max() const{ return m_max; }


    public:
        cloud_ptr whole_cloud;
        cloud_ptr last_cloud;
        std::map<uint32_t,cloud_ptr> clouds;
        Eigen::Matrix4f last_transformation;
        #ifdef CAUV_CLOUD_VISUALISATION
        boost::shared_ptr<pcl::visualization::CloudViewer> viewer;
        static uint32_t viewer_count;
        #endif

    private:
        Eigen::Vector2f m_min;
        Eigen::Vector2f m_max;
        cv::Mat m_vis_buffer;
        Eigen::Vector2f m_vis_origin;
};
#ifdef CAUV_CLOUD_VISUALISATION
uint32_t SonarSLAMImpl::viewer_count = 0;
#endif
} // namespace imgproc
} // namespace cauv


static void drawX(cv::Mat r, int S, float x, float y,
                  Eigen::Vector2f const& min,
                  Eigen::Vector2f const& step,
                  uint8_t weight){
    const float x_idx = 0.5+(x - min[0]) / step[0];
    const float y_idx = 0.5+(y - min[1]) / step[1];
    const Eigen::Vector2i res(r.rows, r.cols);
    for(int dx = -S; dx <= S; dx++)
        for(int dy = -S*(!dx); dy <= S*(!dx); dy++)
            if(dx+x_idx >= 0 && dx+x_idx < res[0] &&
               dy+y_idx >= 0 && dy+y_idx < res[1])
                r.at<uint8_t>(int(dy+y_idx),int(dx+x_idx)) = weight;
}

static cv::Mat renderCloud(
    cloud_t const& cloud,
    Eigen::Vector2f transformed_origin,
    Eigen::Vector2f min, Eigen::Vector2f max,
    Eigen::Vector2i res,
    bool draw_weight = true
){
    if(min[0] > -1) min[0] = -1;
    if(min[1] > -1) min[1] = -1;
    if(max[0] < 1) max[0] = 1;
    if(max[0] < 1) max[0] = 1;

    cv::Mat r(res[1], res[0], CV_8UC1, cv::Scalar(0));
    Eigen::Vector2f step((max[0]-min[0]) / res[0],
                         (max[1]-min[1]) / res[1]);

    float aspect = 1;
    if(res[1] > 0)
        aspect = res[0] / res[1];
    if(step[0] > step[1])
        step[1] = step[0] / aspect;
    else
        step[0] = step[1] * aspect;

    for(size_t i = 0; i < cloud.size(); i++)
        drawX(r, 3, cloud[i].x, cloud[i].y, min, step, draw_weight? cloud[i].getVector4fMap()[3] : 0xff);
    
    drawX(r, 21, cloud.back().x, cloud.back().y, min, step, 0xff);

    // draw axes
    drawX(r, 7, 0, 0, min, step, 64);
    drawX(r, 7, 1, 0, min, step, 64);
    drawX(r, 7, 0, 1, min, step, 64);

    // draw transformed origin
    drawX(r, 9, transformed_origin[0], transformed_origin[1], min, step, 128);

    return r;
}

/*struct CompareKPByIntensity{
    bool operator()(pt_t const& l, pt_t const& r) const{
        return l.getVector4fMap()[3] < r.getVector4fMap()[3];
    }
};*/

static cloud_ptr kpsToCloud(std::vector<KeyPoint> const& kps,
                            Eigen::Matrix4f const& initial_transform,
                            float const& weight_test){

    const Eigen::Vector3f xytheta = xythetaFrom4dAffine(initial_transform);
    const Eigen::Matrix3f rotate  = initial_transform.block<3,3>(0, 0);
    const Eigen::Vector3f translate(xytheta[0], xytheta[1], 0);
    const float rz = xytheta[2];

    debug() << "kpsToCloud: rot:" << rz << "degrees, trans:" << translate[0] <<","<< translate[1];

    cloud_ptr r = boost::make_shared<cloud_t>();
    r->height = 1;
    r->is_dense = true;
    r->reserve(kps.size());

    bool warned_non_planar = false;
    for(size_t i = 0; i < kps.size(); i++){
        if(kps[i].response > weight_test){
            r->points.push_back(pt_t());
            r->descriptors().push_back(kps[i].response);
            pt_t temp(kps[i].pt.x, kps[i].pt.y, 0.0f);
            r->back().getVector3fMap() = rotate * temp.getVector3fMap() + translate;
            //r->back().getVector4fMap()[3] = kps[i].response;
            
            /*
            #warning debug sanity check:
            Eigen::Vector4f sc = initial_transform * Eigen::Vector4f(kps[i].pt.x, kps[i].pt.y, 0, 1);
            Eigen::Vector3f rb = r->back().getVector3fMap();
            if((sc.block<3,1>(0,0) - rb).norm() > 1e-5)
                warning() << "check:"<< sc[0] << "," << sc[1] << "," << sc[2]
                          << " != "  << rb[0] << "," << rb[1] << "," << rb[2];
            */

            if(!warned_non_planar && (*r)[i].z != 0){
                warned_non_planar = true;
                warning() << "non-planar!";
            }
        }
    }
    /*
    #warning hack: ensure cloud includes untransformed origin
    r->push_back(pt_t());
    r->back().getVector3fMap() = rotate * Eigen::Vector3f(0,0,0) + translate;
    */
    r->width = r->size();

    //std::sort(r->rbegin(), r->rend(), CompareKPByIntensity());

    //debug d;
    //d << "sorted kps: {";
    //foreach(pt_t const& pt, *r)
    //    d <<"("<< pt.getVector4fMap()[3] <<":"<< pt.x <<","<< pt.y <<","<< pt.z <<"),";
    //d << "}";

    return r;
}

void SonarSLAMNode::init(){
    m_impl = boost::make_shared<SonarSLAMImpl>();

    // slow node (don't schedule nodes providing input until we've
    // finished doing work here)
    m_speed = slow;

    // inputs (well, parameter input that must be new for the node to be
    // executed):
    registerParamID("keypoints", std::vector<KeyPoint>(), "keypoints used to update map", Must_Be_New);
    registerParamID("delta theta", float(0), "estimated change in orientation (radians) since last image", Must_Be_New);

    // parameters: may be old
    registerInputID("xy image", /*"image to use for map visualisation (may be unconnected)", */Optional);
    registerParamID("xy metres/px", float(1), "range conversion for visualisation image");

    registerParamID("clear", bool(false), "true => discard accumulated point cloud");
    registerParamID("max iters", int(50), "");
    registerParamID("transform eps", float(1e-8), "difference between transforms in successive iters for convergence");
    registerParamID("euclidean fitness", float(1e-6), "max error between consecutive steps for non-convergence");
    registerParamID("reject threshold", float(5), "RANSAC outlier rejection distance");
    registerParamID("max correspond dist", float(5), "");
    registerParamID("score threshold", float(1), "keypoint set will be rejected if mean distance error is greater than this");
    registerParamID("weight test", float(25), "keypoints with weights greater than this will be used for registration");
    registerParamID("feature merge distance", float(0.2), "keypoints closer to each other than this will be merged");
    registerParamID("map merge alpha", float(5), "alpha-hull parameter for map merging");

    registerParamID("-render size", float(400));

    // outputs:
    registerOutputID("whole cloud vis");
    registerOutputID("last added vis");
    registerOutputID("mosaic");
}

Node::out_map_t SonarSLAMNode::doWork(in_image_map_t& inputs){
    out_map_t r;

    bool clear = param<bool>("clear");
    if(clear)
        m_impl->reset();

    const int   max_iters   = param<int>("max iters");
    const float euclidean_fitness   = param<float>("euclidean fitness");
    const float transform_eps       = param<float>("transform eps");
    const float reject_threshold    = param<float>("reject threshold");
    const float max_correspond_dist = param<float>("max correspond dist");
    const float score_thr   = param<float>("score threshold");
    const float delta_theta = param<float>("delta theta");
    const float weight_test = param<float>("weight test");
    const float point_merge_distance = param<float>("feature merge distance");
    const float map_merge_alpha = param<float>("map merge alpha");

    const Eigen::Vector2i render_sz(param<float>("-render size"),param<float>("-render size"));

    const float m_per_pix = param<float>("xy metres/px");
    image_ptr_t xy_image = inputs["xy image"];

    Eigen::Matrix4f delta_transformation = Eigen::Matrix4f::Identity();
    delta_transformation.block<3,3>(0,0) = Eigen::Matrix3f(Eigen::AngleAxisf(delta_theta, Eigen::Vector3f::UnitZ()));

    Eigen::Matrix4f guess = m_impl->last_transformation * delta_transformation;
    // squish the z translation
    guess(2,3) = 0;
    //guess(0,2) = 0; guess(1,2) = 0;
    //guess(2,0) = 0; guess(2,1) = 0;

    // weight-test throwing away low scorers:
    cloud_ptr new_cloud = kpsToCloud(
        param< std::vector<KeyPoint> >("keypoints"),
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
            m_impl->addNewCloud(final, point_merge_distance, map_merge_alpha);
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

    return r;
}

