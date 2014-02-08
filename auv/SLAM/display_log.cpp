#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <utility/options.h>
#include <cauv_slam/SonarImage.h>

#include "sonar_ops.h"

int main(int argc, char **argv) {
    cauv::Options options("Manual positioning of sonar images");
    namespace po = boost::program_options;
    options.desc.add_options()
        ("directory,d", po::value<std::string>()->required(), "Directory of sonar images to load")
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
    auto files = get_msg_files(image_directory);
    int i = 0;
    for (auto &file: files) {
        if (i++ < start_frame) { continue; }
        if (i > end_frame) { break; }
        std::cout << file << std::endl;
        cauv_slam::SonarImage image;
        load_sonar_image(file, image);
        std::cout << "Range: " << image.rangeEnd << "(" << image.rangeConversion << "m per bin)" << std::endl;
        auto map_mat = get_polar_to_cartesian_map(image.rangeStart, image.rangeEnd,
                                                  image.rangeConversion, image.bearings,
                                                  10);
        auto mat = sonar_msg_to_mat(image);
        if (!(i % 10) && false) {
            mat = cv::Mat(mat.rows, mat.cols, CV_8UC1, 255);
        }
        cv::Mat cartesian_mat(map_mat.rows, map_mat.cols, CV_8UC1);
        cv::remap(mat, cartesian_mat, map_mat, cv::Mat(), CV_INTER_LINEAR);
        cv::imshow("image", mat);
        cv::imshow("map", cartesian_mat);
        std::cout << image.image.size() << " " << image.timestamp << std::endl;
        cv::waitKey(100);
    }
    return 0;
}
