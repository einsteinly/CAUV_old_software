#include <deque>
#include <memory>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <pointmatcher/PointMatcher.h>

#include <utility/options.h>
#include <utility/make_unique.h>

#include <cauv_slam/SLAMMap.h>
#include <cauv_slam/SonarImage.h>
#include <cauv_slam/IncrementalPose.h>

#include "sonar_ops.h"
#include "corner_feature_detector.h" 
#include "ros_save.hpp"

using namespace cauv;

namespace IncrementalPose {
    static cauv_slam::IncrementalPose from3dAffine(Eigen::MatrixXf matrix);
};

cauv_slam::IncrementalPose IncrementalPose::from3dAffine(Eigen::MatrixXf matrix) {
    Eigen::Vector3f point_a(0,0,1);
    Eigen::Vector3f point_b(1,0,1);
    point_a = matrix * point_a;
    point_b = matrix * point_b;
    point_a /= point_a(2);
    point_b /= point_b(2);
    auto delta = point_b - point_a;
    float rotation = std::atan2(delta(1), delta(0));
    cauv_slam::IncrementalPose pose;
    pose.dx = point_a(0); pose.dy = point_a(1); pose.dtheta = rotation;
    return pose;
}

std::unique_ptr<LocalFeatureDetector> getDetector(boost::program_options::variables_map &vm) {
    if (vm.count("features")) {
        std::string features = vm["features"].as<std::string>();
        if (features == "FAST") {
            return cauv::make_unique<CornerFeatureDetector>();
        } else {
            throw std::runtime_error("Unknown feature detector " + features);
        }
    }
    return std::unique_ptr<LocalFeatureDetector>();
}

typedef PointMatcher<float> PM;

std::unique_ptr<PM::DataPoints>
features_to_datapoints(std::vector<LocalPolarFeature> &features, PolarMapping &mapping) {
    Eigen::MatrixXf feature_matrix;
    int n_features = features.size();
    feature_matrix.resize(3, n_features);
    for (int i = 0; i < n_features; i++) {
        auto cart_feature = features[i].toCart(mapping);
        feature_matrix(0, i) = cart_feature.x;
        feature_matrix(1, i) = cart_feature.y;
        feature_matrix(2, i) = 1.0;
    }
    PM::DataPoints::Label label("corners", 2);
    PM::DataPoints::Labels labels;
    labels.push_back(label);
    return cauv::make_unique<PM::DataPoints>(feature_matrix, labels);
}

void show_points(cv::Mat mat, PM::DataPoints &datapoints, cv::Scalar colour, float scale, float range) {
    int n_points = datapoints.features.cols();
    for (int i = 0; i < n_points; i++) {
        float x = datapoints.features(0, i);
        float y = datapoints.features(1, i);
        float w = datapoints.features(2, i);
        x /= w; y /= w;
        cv::circle(mat, cv::Point((-y + range)*scale, x * scale), 2, colour);
    } 
}

int main(int argc, char **argv) {
    cauv::Options options("Benchmark scan matching");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("directory,d", po::value<std::string>()->required(), "Directory of sonar images to load")
        ("features,f", po::value<std::string>()->required(), "Features to use for matching")
        ("map,m", po::value<std::string>(), "Map file to output to")
        ("vis,v", po::value<bool>()->zero_tokens(), "Show visualisation of point cloud matches")
        ("start,s", po::value<int>()->default_value(0), "Start frame")
        ("end,e", po::value<int>()->default_value(10000000), "End frame")
      ;
    if (options.parseOptions(argc, argv)) {
        return 0;
    };

    auto vm = options.vm;

    auto image_directory = vm["directory"].as<std::string>();

    std::cout << vm["directory"].as<std::string>() << std::endl;
    int start_frame = vm["start"].as<int>();
    int end_frame = vm["end"].as<int>();
    bool show_vis = vm.count("vis");
    auto feature_detector = getDetector(vm);

    auto files = get_msg_files(image_directory);

    cauv_slam::SLAMMap map_message;
    bool save_map = vm.count("map");
    int i = 0;
    std::deque<std::unique_ptr<PM::DataPoints>> recent_data_points;
    for (auto &file: files) {
        if (i++ < start_frame) { continue; }
        if (i > end_frame) { break; }
        std::cout << file << std::endl;
        cauv_slam::SonarImage image;
        load_sonar_image(file, image);
        auto mapping = get_polar_mapping(image);
        auto mat = sonar_msg_to_mat(image);

        auto features = feature_detector->extractFeatures(mat);
        auto data_points = features_to_datapoints(features, mapping);
        if (recent_data_points.size() < 1) {
            recent_data_points.push_back(std::move(data_points));
            continue;
        }
        cv::Mat vis_mat;
        if (show_vis) { 
            vis_mat = cv::Mat::zeros(350, 700, CV_8UC3);
            show_points(vis_mat, *data_points, cv::Scalar(0xff, 0xff, 0xff), 10, 35);
        }
        for (auto &past_data_points : recent_data_points) {
            PM::ICP icp;
            icp.setDefault();
            auto params = icp(*past_data_points, *data_points);

            auto pose = IncrementalPose::from3dAffine(params);
            pose.scan_id = i;
            std::cout << pose << std::endl;
            if (save_map) {
                map_message.poses.push_back(pose);
            }

            if (show_vis) {
                PM::DataPoints matched_points(*past_data_points);
                icp.transformations.apply(matched_points, params);
                show_points(vis_mat, matched_points, cv::Scalar(0x44, 0x44, 0xff), 10, 35);
            }
        }
        if (show_vis) {
            cv::imshow("match", vis_mat);
            cv::waitKey();
        }
        recent_data_points.push_back(std::move(data_points));
        recent_data_points.pop_front();
    }
    if (save_map) {
        cauv::save_message(vm["map"].as<std::string>(), map_message);
    }
    return 0;
}
