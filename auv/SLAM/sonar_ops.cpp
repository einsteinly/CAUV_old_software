#include <fstream>
#include <string>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include <cmath>
#include <boost/math/constants/constants.hpp>

#include <boost/filesystem.hpp>
#include <boost/range/algorithm.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cauv_slam/SonarImage.h>

#include "sonar_ops.h"
#include "ros_save.hpp"

using namespace cauv;

namespace fs = boost::filesystem;

PolarMapping
cauv::PolarMapping::generate(float rangeStart, float rangeEnd, float rangeConversion) {
    std::vector<int> bearings;
    static const int num_beams = 256;
    std::vector<int32_t> r;
    const double radConvert = (180.0 / M_PI) * (6400.0/360.0) * 0x10000;
    for(int beam = 0; beam <= num_beams; beam++){
        // see Gemini Interface Specification Appendix B
        double radians = std::asin(((2*(beam+0.5) - num_beams) / num_beams) * 0.86602540);
        bearings.push_back(round(radConvert * radians));
    }
    return PolarMapping{rangeStart, rangeEnd, rangeConversion, bearings};
};

cauv::LocalCartFeature
cauv::LocalPolarFeature::toCart(cauv::PolarMapping &m) {
    static const float pi = boost::math::constants::pi<float>();
    //TODO: interpolate bearing
    int int_bearing = m.bearings.at(int(bearing));
    float actual_bearing = int_bearing / (6400.0 * 0x10000) * 2.0 * pi;
    float actual_range = range * m.rangeConversion + m.rangeStart;
    return LocalCartFeature{std::cos(actual_bearing) * actual_range,
                            std::sin(actual_bearing) * actual_range,
                            colour};
}

std::vector<fs::path> cauv::get_msg_files(std::string directory) {
    std::vector<fs::path> files;
    std::copy(fs::directory_iterator(directory), fs::directory_iterator(),
                std::back_inserter(files));
    auto end_it = std::remove_if(
                   files.begin(),
                   files.end(),
                   [](fs::path &file) -> bool {
                       return fs::is_regular_file(file) && file.extension() == "rosmsg";
                   });
    files.erase(end_it, files.end());
    std::sort(files.begin(), files.end());
    return files;
}

void cauv::load_sonar_image(fs::path file, cauv_slam::SonarImage &image) {
    cauv::load_message(file.c_str(), image);
}

cv::Mat cauv::sonar_msg_to_mat(cauv_slam::SonarImage& image) {
    int width = image.bearings.size() - 1;
    int height = std::floor(0.5 + (image.rangeEnd - image.rangeStart) / image.rangeConversion);
    return cv::Mat(height, width, CV_8UC1, &image.image[0]);
}

cauv::PolarMapping cauv::get_polar_mapping(cauv_slam::SonarImage &image) {
    return PolarMapping {image.rangeStart, image.rangeEnd,
                         image.rangeConversion, image.bearings};
}

static cv::Vec2f get_bearing_range(const float x, const float y,
                                   PolarMapping &m) {

    static const float pi = boost::math::constants::pi<float>();
    float range = std::sqrt(y*y + x*x);
    if (range > m.rangeEnd || range < m.rangeStart) {
        return cv::Vec2f(-1.0, -1.0);
    }
    size_t n_bearings = m.bearings.size();
    float bearing = (std::atan2(y, x) - pi / 2.0) / 2.0 / pi * 6400.0 * 0x10000;
    for (size_t i = 0; i < n_bearings; i++) {
        if (m.bearings[i] > bearing) {
            if (i == 0) {
                return cv::Vec2f(-1.0, -1.0);
            }
            if (i == n_bearings - 1) {
                return cv::Vec2f(-1.0, -1.0);
            }
            bearing = (float)i + (bearing - m.bearings[i - 1]) / (m.bearings[i] - m.bearings[i - 1]); 
            return cv::Vec2f(bearing, range - m.rangeStart);
        }
    }
    return cv::Vec2f(-1.0, -1.0);
}

cv::Mat cauv::get_polar_to_cartesian_map(PolarMapping &m, float scale) {
    int width = m.rangeEnd * 2 * scale;
    int height = m.rangeEnd * scale;
    cv::Mat map(height, width, CV_32FC2);
    for (int ii = 0; ii < width; ii++) {
        for (int jj = 0; jj < height; jj++) {
            auto point = get_bearing_range((ii - width / 2) / scale,
                                           jj / scale, m);
            point[1] /= m.rangeConversion;
            map.at<cv::Vec2f>(jj, ii) = point;
        }
    }
    return map;
}
