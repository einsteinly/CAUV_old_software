#pragma once
#include <vector>
#include <string>

#include <boost/filesystem.hpp>

#include <opencv2/core/core.hpp>

#include <cauv_slam/SonarImage.h>

namespace cauv {

struct Pose {
    float x;
    float y;
    float bearing;
    unsigned int scan_id;
};

struct PolarMapping {
    static PolarMapping generate(float rangeStart = 0, float rangeEnd = 35, float rangeConversion = 0.0341797);
    float rangeStart;
    float rangeEnd;
    float rangeConversion;
    std::vector<int> bearings;
};

struct LocalCartFeature {
    float x; //Perpendicular to AUV's axis
    float y; //Parallel to AUV's axis
    float colour;
};

struct LocalPolarFeature {
    float bearing; //Forward = 0
    float range;
    float colour; //Any extra 'dimension' or information attached to the feature to distinguish it
    LocalCartFeature toCart(PolarMapping &m);
};

struct GlobalCartFeature {
    float x;
    float y;
    cv::Mat descriptor;
};

std::vector<boost::filesystem::path> get_msg_files(std::string directory);

void load_sonar_image(boost::filesystem::path file, cauv_slam::SonarImage &image);

PolarMapping get_polar_mapping(cauv_slam::SonarImage &image);

cv::Mat sonar_msg_to_mat(cauv_slam::SonarImage& image);

cv::Mat get_polar_to_cartesian_map(PolarMapping &m, float scale);

}
