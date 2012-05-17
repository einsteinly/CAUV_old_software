/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#include <boost/algorithm/string.hpp>
 
#include <pcl/point_types.h>

#include <debug/cauv_debug.h>

#include "slamCloud.h"
#include "slamCloudPart.h"
#include "scanMatching.h"
#include "graphOptimiser.h"

#include <generated/types/TimeStamp.h>
#include <generated/types/KeyPoint.h>

namespace ci = cauv::imgproc;

typedef pcl::PointXYZ pt_t;
typedef ci::SlamCloudPart<pt_t> cloud_t;
typedef cloud_t::Ptr cloud_ptr;
typedef boost::shared_ptr<ci::SlamCloudLocation> location_ptr;

typedef std::vector<cauv::KeyPoint> kp_vec;

int Graph_Iters = 10;
float Overlap_Threshold = 0.2;
float Keyframe_Spacing = 2;
float Min_Initial_Points = 3;
float Good_Keypoint_Distance = 0.1;
int Max_Iters = 20;
float Euclidean_Fitness = 1e-7;
float Transform_Eps = 1e-9;
float Reject_Threshold = 0.5;
float Max_Correspond_Dist = 1.1;
float Score_Thr = 22;
float Ransac_Iters = 0;
std::string Match_Algorithm = "NL-ICP";

namespace po = boost::program_options;

int main(int argc, char** argv){
    po::options_description desc("Options:");
    desc.add_options()
        ("help", "produce help message")
        ("verbose,v", po::value<int>(), "verbosity")
        ("graph-iters", po::value<int>(), "graph optimisation iterations")
        ("overlap-threshold", po::value<float>(), "proportional area overlap to be considered overlapped")
        ("keyframe-spacing", po::value<float>(), "minimum spacing between keyframes")
        ("min-initial-points", po::value<float>(), "minimum number of points for first cloud")
        ("good-keypoint-distance", po::value<float>(), "minimum number of points for first cloud")
        ("max-iters", po::value<int>(), "max scan-alignment iterations")
        ("euclidean-fitness", po::value<float>(), "")
        ("transform-epsilon", po::value<float>(), "")
        ("reject-threshold", po::value<float>(), "")
        ("max-correspond-dist", po::value<float>(), "")
        ("score-threshold", po::value<float>(), "")
        ("match-algorithm", po::value<std::string>(), "")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if(vm.count("help")){
        std::cout << desc << "\n";
        return 1;
    }

    if(vm.count("verbose")) debug::setLevel(vm["verbose"].as<int>());
    if(vm.count("graph-iters"))         Graph_Iters         = vm["graph-iters"].as<int>(); 
    if(vm.count("overlap-threshold"))   Overlap_Threshold   = vm["overlap-threshold"].as<float>();
    if(vm.count("keyframe-spacing"))    Keyframe_Spacing    = vm["keyframe-spacing"].as<float>();
    if(vm.count("min-initial-points"))  Min_Initial_Points  = vm["min-initial-points"].as<float>();
    if(vm.count("good-keypoint-distance")) Good_Keypoint_Distance = vm["good-keypoint-distance"].as<float>();
    if(vm.count("max-iters"))           Max_Iters           = vm["max-iters"].as<int>();
    if(vm.count("euclidean-fitness"))   Euclidean_Fitness   = vm["euclidean-fitness"].as<float>();
    if(vm.count("transform-epsilon"))   Transform_Eps       = vm["transform-epsilon"].as<float>();
    if(vm.count("reject-threshold"))    Reject_Threshold    = vm["reject-threshold"].as<float>();
    if(vm.count("max-correspond-dist")) Max_Correspond_Dist = vm["max-correspond-dist"].as<float>();
    if(vm.count("score-threshold"))     Score_Thr           = vm["score-threshold"].as<float>();
    if(vm.count("match-algorithm"))     Match_Algorithm     = vm["match-algorithm"].as<std::string>();

    ci::SlamCloudGraph<pt_t> g;
    g.setParams(
        Overlap_Threshold, Keyframe_Spacing, Min_Initial_Points,
        Good_Keypoint_Distance
    );

    cauv::TimeStamp t_a(0, 0);
    cauv::TimeStamp t_b(1, 0);
    cauv::TimeStamp t_c(3, 0);
    cauv::TimeStamp t_d(4, 0);
    cauv::TimeStamp t_e(5, 0);
    kp_vec kps_a;
    kps_a.push_back(cauv::KeyPoint(cauv::floatXY(0,0), 1, 0, 1, 0, 0));
    kps_a.push_back(cauv::KeyPoint(cauv::floatXY(0,5), 1, 0, 1, 0, 0));
    kps_a.push_back(cauv::KeyPoint(cauv::floatXY(5,5), 1, 0, 1, 0, 0));
    kps_a.push_back(cauv::KeyPoint(cauv::floatXY(5,0), 1, 0, 1, 0, 0));
    kp_vec kps_b;
    kps_b.push_back(cauv::KeyPoint(cauv::floatXY(1,0), 1, 0, 1, 0, 0));
    kps_b.push_back(cauv::KeyPoint(cauv::floatXY(1,5), 1, 0, 1, 0, 0));
    kps_b.push_back(cauv::KeyPoint(cauv::floatXY(6,5), 1, 0, 1, 0, 0));
    kps_b.push_back(cauv::KeyPoint(cauv::floatXY(6,0), 1, 0, 1, 0, 0));
    kp_vec kps_c;
    kps_c.push_back(cauv::KeyPoint(cauv::floatXY(3,1), 1, 0, 1, 0, 0));
    kps_c.push_back(cauv::KeyPoint(cauv::floatXY(3,6), 1, 0, 1, 0, 0));
    kps_c.push_back(cauv::KeyPoint(cauv::floatXY(8,6), 1, 0, 1, 0, 0));
    kps_c.push_back(cauv::KeyPoint(cauv::floatXY(8,1), 1, 0, 1, 0, 0));
    // rotated about 11 degrees about origin, shifted (-2, -1)
    kp_vec kps_d;
    kps_d.push_back(cauv::KeyPoint(cauv::floatXY(-2,-1), 1, 0, 1, 0, 0));
    kps_d.push_back(cauv::KeyPoint(cauv::floatXY(-3,4), 1, 0, 1, 0, 0));
    kps_d.push_back(cauv::KeyPoint(cauv::floatXY(2,5), 1, 0, 1, 0, 0));
    kps_d.push_back(cauv::KeyPoint(cauv::floatXY(3,0), 1, 0, 1, 0, 0));
    // rotated about -11 degrees, shifted -2, 0
    kp_vec kps_e;
    kps_e.push_back(cauv::KeyPoint(cauv::floatXY(-2,0), 1, 0, 1, 0, 0));
    kps_e.push_back(cauv::KeyPoint(cauv::floatXY(-1,5), 1, 0, 1, 0, 0));
    kps_e.push_back(cauv::KeyPoint(cauv::floatXY(4,4), 1, 0, 1, 0, 0));
    kps_e.push_back(cauv::KeyPoint(cauv::floatXY(3,-1), 1, 0, 1, 0, 0));

    std::vector<cloud_ptr> test_clouds;
    test_clouds.push_back(boost::make_shared<cloud_t>(kps_a, t_a, cloud_t::FilterResponse(0)));
    test_clouds.push_back(boost::make_shared<cloud_t>(kps_b, t_b, cloud_t::FilterResponse(0)));
    test_clouds.push_back(boost::make_shared<cloud_t>(kps_c, t_c, cloud_t::FilterResponse(0)));
    test_clouds.push_back(boost::make_shared<cloud_t>(kps_d, t_d, cloud_t::FilterResponse(0)));
    test_clouds.push_back(boost::make_shared<cloud_t>(kps_e, t_e, cloud_t::FilterResponse(0)));

    ci::GraphOptimiserV1 graph_optimiser(Graph_Iters);
    boost::shared_ptr< ci::PairwiseMatcher<pt_t> > scan_matcher;
    scan_matcher = ci::makeICPNonLinearPairwiseMatcherShared(
        Max_Iters, Euclidean_Fitness, Transform_Eps,
        Reject_Threshold, Max_Correspond_Dist, Score_Thr,
        Ransac_Iters
    );
    if(boost::iequals(Match_Algorithm, "NDT")){
        scan_matcher = ci::makeNDTPairwiseMatcherShared(
            Max_Iters, Euclidean_Fitness, Transform_Eps, Score_Thr
        );
    }else if(boost::iequals(Match_Algorithm, "ICP")){
        scan_matcher = ci::makeICPPairwiseMatcherShared(
            Max_Iters, Euclidean_Fitness, Transform_Eps,
            Reject_Threshold, Max_Correspond_Dist, Score_Thr,
            Ransac_Iters
        );
    }else if(boost::iequals(Match_Algorithm, "NL-ICP")){
        scan_matcher = ci::makeICPNonLinearPairwiseMatcherShared(
            Max_Iters, Euclidean_Fitness, Transform_Eps,
            Reject_Threshold, Max_Correspond_Dist, Score_Thr,
            Ransac_Iters
        );
    }else{
        error() << "invalid scan matching algorithm:" << Match_Algorithm
                << "valid options are: NDT, ICP, NL-ICP";
        return 1;
    }
    
    foreach(cloud_ptr const& scan, test_clouds){
        {
            debug d;
            d << "cloud:";
            for(std::size_t j = 0; j < scan->size(); j++)
                d << "(" << (*scan)[j].getVector3fMap().transpose() << ")";
        }

        Eigen::Matrix4f global_transformation = Eigen::Matrix4f::Zero();
        /*
        const float delta_theta = 0;
        Eigen::Matrix4f relative_transformation_guess = Eigen::Matrix4f::Identity();
        relative_transformation_guess.block<3,3>(0,0) = Eigen::Matrix3f(
            Eigen::AngleAxisf(delta_theta, Eigen::Vector3f::UnitZ())
        );

        Eigen::Matrix4f guess = g.guessTransformationAtTime(scan->time());
        // use rotation from external guess, and translation from internal guess:
        guess.block<3,3>(0,0) = relative_transformation_guess.block<3,3>(0,0);
        */
        const float confidence = g.registerScan(
            scan, Eigen::Matrix4f::Identity(), *scan_matcher, graph_optimiser, global_transformation
        );

        debug() << "matched (confidence=" << confidence << ") at:\n" << global_transformation;
    }
    
    info() << "results:";
    foreach(location_ptr const& loc, g.allScans()){
        const Eigen::Matrix4f a = loc->globalTransform();
        const Eigen::Matrix2f r = a.block<2,2>(0, 0);
        const Eigen::Vector2f t = a.block<2,1>(0, 3);
        const Eigen::Vector2f tmp = r * Eigen::Vector2f(1,0);
        const float rz = std::atan2(tmp[1], tmp[0]);
        info() << "x,y=" << t[0] << "," << t[1] << "r=" << rz * 180/M_PI << "deg.";//\n" << a;
    }
}

