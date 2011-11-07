#include "sonarSLAMNode.h"

#include <pcl/point_types.h>
//#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/common/transforms.h>
#ifdef CAUV_CLOUD_DUMP
#include <pcl/io/pcd_io.h>
#endif
#ifdef CAUV_CLOUD_VISUALISATION
#include <pcl/visualization/cloud_viewer.h>
#endif

#include <utility/bash_cout.h>

namespace cauv{
namespace imgproc{
// register this node type in the factory thing that creates nodes
DEFINE_NFR(SonarSLAMNode, NodeType::SonarSLAM);

} // namespace imgproc
} // namespace cauv

using namespace cauv;
using namespace cauv::imgproc;

typedef pcl::PointXYZ pt_t;
typedef pcl::PointCloud<pt_t> cloud_t;
// actually a boost shared_ptr:
typedef cloud_t::Ptr cloud_ptr;

static uint32_t Min_Initial_Points = 10;

// avoid including PCL stuff in the header:
namespace cauv{
namespace imgproc{
struct SonarSLAMImpl{
    cloud_ptr whole_cloud;
    std::map<uint32_t,cloud_ptr> clouds;
    Eigen::Matrix4f last_transformation;
    #ifdef CAUV_CLOUD_VISUALISATION
    boost::shared_ptr<pcl::visualization::CloudViewer> viewer;
    static uint32_t viewer_count;
    #endif
};
#ifdef CAUV_CLOUD_VISUALISATION
uint32_t SonarSLAMImpl::viewer_count = 0;
#endif
} // namespace imgproc
} // namespace cauv

static cloud_ptr kpsToCloud(std::vector<KeyPoint> const& kps, Eigen::Matrix4f const& initial_transform){
    // split transform into affine parts:
    Eigen::Matrix3f rotate    = initial_transform.block<3,3>(0, 0);
    Eigen::Vector3f translate = initial_transform.block<3,1>(0, 3);
    
    Eigen::Vector3f t = rotate*Eigen::Vector3f(1,0,0);
    float rz = 180*M_PI*std::atan2(t[1], t[0]);
    debug() << "kpsToCloud: rot:" << rz << "degrees, trans:" << translate[0] <<","<< translate[1];

    cloud_ptr r = boost::make_shared<cloud_t>();
    r->height = 1;
    r->width = kps.size();
    r->is_dense = false; // TODO: true?
    r->points.resize(kps.size());
    
    bool warned_non_planar = false;
    for(size_t i = 0; i < kps.size(); i++){
        pt_t temp(kps[i].pt.x, kps[i].pt.y, 0);
        r->points[i].getVector3fMap() = rotate * temp.getVector3fMap() + translate;
        if(!warned_non_planar && r->points[i].z != 0){
            warned_non_planar = true;
            warning() << "non-planar!";
        }
    }

    return r;
}

void SonarSLAMNode::init(){
    m_impl = boost::make_shared<SonarSLAMImpl>();
    m_impl->last_transformation = Eigen::Matrix4f::Identity();
    
    #ifdef CAUV_CLOUD_VISUALISATION
    m_impl->viewer_count++;
    m_impl->viewer = boost::make_shared<pcl::visualization::CloudViewer>(mkStr() << "SonarSLAM " << m_impl->viewer_count);
    #endif

    // slow node (don't schedule nodes providing input until we've
    // finished doing work here)
    m_speed = slow;

    // input (well, parameter input that must be new for the node to be
    // executed):
    registerParamID("keypoints", std::vector<KeyPoint>(), "keypoints used to update map", Must_Be_New);
    registerParamID("clear", bool(false), "true => discard accumulated point cloud");
    registerParamID("max iters", int(500), "");
    registerParamID("transform eps", float(1e-8), "difference between transforms in successive iters for convergence");
    registerParamID("euclidean fitness", float(1e-6), "max error between consecutive steps for non-convergence");
    registerParamID("reject threshold", float(5), "RANSAC outlier rejection distance");
    registerParamID("max correspond dist", float(5), "");
    registerParamID("score threshold", float(1), "keypoint set will be rejected if mean distance error is greater than this");
    
    // outputs:
    //registerOutputID<image_ptr_t>("image_out");
    
    // parameters:
    //registerParamID<float>("some scalar param", 1.0f, "...");
}

Node::out_map_t SonarSLAMNode::doWork(in_image_map_t& inputs){
    out_map_t r;

    bool clear = param<bool>("clear");
    if(clear)
        m_impl->whole_cloud.reset();
    
    int max_iters = param<int>("max iters");
    float euclidean_fitness = param<float>("euclidean fitness");
    float transform_eps = param<float>("transform eps");
    float reject_threshold = param<float>("reject threshold");
    float max_correspond_dist = param<float>("max correspond dist");
    float score_thr = param<float>("score threshold");
    
    cloud_ptr new_cloud = kpsToCloud(
        param< std::vector<KeyPoint> >("keypoints"),
        m_impl->last_transformation
    );
    // TODO: propagate sonar timestamp with keypoints... somehow... and use
    // that instead of an index
    static uint32_t cloud_num = 0;
    m_impl->clouds[++cloud_num] = new_cloud;
    
    if(!m_impl->whole_cloud){
        if(new_cloud->points.size() > Min_Initial_Points){
            info() << BashColour::Green
                   << "new cloud with" << new_cloud->points.size() << "points";
            m_impl->whole_cloud = boost::make_shared<cloud_t>(*new_cloud);
            debug(3) << "transformation is:\n" << m_impl->last_transformation;
        }else{
            debug() << "not enough points to initialise whole cloud";
        }
    }else{
        debug() << "trying to match" << new_cloud->points.size() << "points to cloud";
        /*
        *m_impl->whole_cloud += *new_cloud;
        pcl::io::savePCDFile("sonarSLAM.pcd", *m_impl->whole_cloud);
        */
        
        pcl::IterativeClosestPointNonLinear<pt_t,pt_t> icp;
        icp.setInputCloud(new_cloud);
        icp.setInputTarget(m_impl->whole_cloud);

        icp.setMaxCorrespondenceDistance(max_correspond_dist);
        icp.setMaximumIterations(max_iters);
        icp.setTransformationEpsilon(transform_eps);
        icp.setEuclideanFitnessEpsilon(euclidean_fitness);
        icp.setRANSACOutlierRejectionThreshold(reject_threshold);

        cloud_t final;
        icp.align(final);
        Eigen::Matrix4f final_transform = icp.getFinalTransformation();
        float score = icp.getFitnessScore();
        info() << BashColour::Green
               << "converged:" << icp.hasConverged()
               << "score:" << score;
        debug(3) << "transformation:\n" << final_transform;
        if(icp.hasConverged() && score < score_thr){
            m_impl->last_transformation = final_transform;
            *(m_impl->whole_cloud) += final;
            Eigen::Matrix3f r = final_transform.block<3,3>(0, 0);
            Eigen::Vector3f t = final_transform.block<3,1>(0, 3);
            info() << BashColour::Green
                   << "added transformed points at: " << t[0] << "," << t[1] << "," << t[2];
            
            #ifdef CAUV_CLOUD_VISUALISATION
            m_impl->viewer->showCloud(m_impl->whole_cloud);
            #endif

            #ifdef CAUV_CLOUD_DUMP
            static uint32_t n = 0;
            pcl::io::savePCDFile(mkStr() << "sonarSLAM" << n++ << ".pcd", *m_impl->whole_cloud);
            pcl::io::savePCDFile(mkStr() << "sonarSLAM-scan" << n++ << ".pcd", *new_cloud);
            pcl::io::savePCDFile(mkStr() << "sonarSLAM.pcd",  *m_impl->whole_cloud);
            #endif
            
        }else if(score >= score_thr){
            info() << BashColour::Green
                   << "points will not be added to the whole cloud (error too high)";
        }else{
            info() << BashColour::Green
                   << "points will not be added to the whole cloud (not converged)";
        }
    }


    return r;
}

