#include <iostream>
#include <memory>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <utility/options.h>
#include <utility/make_unique.h>
#include <cauv_slam/SonarImage.h>

#include "sonar_ops.h"
#include "corner_feature_detector.h" 

int main(int argc, char **argv) {
    cauv::Options options("Playback sonar log");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("directory,d", po::value<std::string>()->required(), "Directory of sonar images to load")
        ("features,f", po::value<std::string>(), "Feature detection to display")
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
    using namespace cauv;
    std::unique_ptr<LocalFeatureExtractor> feature_detector;
    if (vm.count("features")) {
        if (vm["features"].as<std::string>() == "FAST") {
            feature_detector = cauv::make_unique<CornerFeatureDetector>();
        }
    }
    auto files = get_msg_files(image_directory);
    int i = 0;
    for (auto &file: files) {
        if (i++ < start_frame) { continue; }
        if (i > end_frame) { break; }
        std::cout << file << std::endl;
        cauv_slam::SonarImage image;
        load_sonar_image(file, image);
        std::cout << "Range: " << image.rangeEnd << "(" << image.rangeConversion << "m per bin)" << std::endl;
        auto polar_mapping = get_polar_mapping(image);
        auto map_mat = get_polar_to_cartesian_map(polar_mapping, 10);
        auto mat = sonar_msg_to_mat(image);
        std::cout << map_mat.rows << " " << map_mat.cols << std::endl;
        cv::Mat cartesian_mat(map_mat.rows, map_mat.cols, CV_8UC1);
        cv::remap(mat, cartesian_mat, map_mat, cv::Mat(), CV_INTER_LINEAR);

        if (feature_detector) {
            auto features = feature_detector->extractFeatures(mat);
            for (auto &feature : features) {
                cv::circle(mat, cv::Point(feature.bearing, feature.range), 2, 0xff);
                auto cart_point = feature.toCart(polar_mapping);
                cv::circle(cartesian_mat, cv::Point((-cart_point.y + image.rangeEnd)*10, cart_point.x * 10), 2, 0xff);
            }
        }
        cv::imshow("image", mat);
        cv::imshow("map", cartesian_mat);
        std::cout << image.image.size() << " " << image.timestamp << std::endl;
        cv::waitKey(100);
    }
    return 0;
}
