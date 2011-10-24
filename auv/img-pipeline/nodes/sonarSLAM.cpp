#include "sonarSLAMNode.h"

#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#ifdef CAUV_CLOUD_VISUALISATION
#include <pcl/visualization/cloud_viewer.h>
#endif

using namespace cauv;
using namespace cauv::imgproc;

typedef pcl::PointXYZ pt_t;
typedef pcl::PointCloud<pt_t> cloud_t;
// actually a boost shared_ptr (I think...):
typedef cloud_t::Ptr cloud_ptr;

static uint32_t Min_Initial_Points = 50;

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

    debug() << "kpsToCloud: rot:\n" << rotate << "\ntrans:\n" << translate;

    cloud_ptr r = boost::make_shared<cloud_t>();
    r->height = 1;
    r->width = kps.size();
    r->is_dense = false; // TODO: true?
    r->points.resize(kps.size());

    for(size_t i = 0; i < kps.size(); i++){
        pt_t temp(kps[i].pt.x, kps[i].pt.y, 0);
        r->points[i].getVector3fMap() = rotate * temp.getVector3fMap() + translate;
        if(r->points[i].z != 0)
            warning() << "non-planar!";
    }

    return r;
}

void SonarSLAMNode::init(){
    m_impl = boost::make_shared<SonarSLAMImpl>();
    m_impl->last_transformation << 1,0,0,0,
                                   0,1,0,0,
                                   0,0,1,0,
                                   0,0,0,1;
    
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
    
    // outputs:
    //registerOutputID<image_ptr_t>("image_out");
    
    // parameters:
    //registerParamID<float>("some scalar param", 1.0f, "...");
}

Node::out_map_t SonarSLAMNode::doWork(in_image_map_t& inputs){
    out_map_t r;
    
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
            debug() << "new cloud with" << new_cloud->points.size() << "points";
            m_impl->whole_cloud = boost::make_shared<cloud_t>(*new_cloud);
            debug() << "transformation is:\n" << m_impl->last_transformation;
        }else{
            debug() << "not enough points to initialise whole cloud";
        }
    }else{
        debug() << "trying to match" << new_cloud->points.size() << "points to cloud";
        pcl::IterativeClosestPoint<pt_t,pt_t> icp;
        icp.setInputCloud(new_cloud);
        icp.setInputTarget(m_impl->whole_cloud);

        icp.setMaxCorrespondenceDistance(0.1);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-6); // difference between transforms in successive iters for convergence
        icp.setEuclideanFitnessEpsilon(1);  // max error between consecutive steps for non-convergence

        cloud_t final;
        icp.align(final);
        Eigen::Matrix4f final_transform = icp.getFinalTransformation();
        debug() << "converged:" << icp.hasConverged()
                << "score:" << icp.getFitnessScore()
                << "transformation:\n" << final_transform;
        if(icp.hasConverged()){
            m_impl->last_transformation = final_transform;
            *(m_impl->whole_cloud) += final;
            debug() << "added transformed points to the cloud";
            Eigen::Matrix3f r = final_transform.block<3,3>(0, 0);
            Eigen::Vector3f t = final_transform.block<3,1>(0, 3);
            debug() << "position: " << t[0] << "," << t[1] << "," << t[2];
            
            #ifdef CAUV_CLOUD_VISUALISATION
            m_impl->viewer->showCloud(m_impl->whole_cloud);
            #endif
        }else{
            debug() << "points will not be added to the whole cloud (not converged)";
        }
    }


    return r;
}

