#include "surf_extractor.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <algorithm>

using namespace cauv;

std::vector<GlobalCartFeature>
SURFExtractor::extractFeatures(cv::Mat polarImage, PolarMapping &mapping) {
    auto detector = cv::SURF(300, 4, 2, false, true);
    const float scale = 10;
    auto map_mat = get_polar_to_cartesian_map(mapping, scale);
    cv::Mat cartesian_mat(map_mat.rows, map_mat.cols, CV_8UC1);
    cv::remap(polarImage, cartesian_mat, map_mat, cv::Mat(), CV_INTER_LINEAR);
    cv::Mat descriptors;
    std::vector<cv::KeyPoint> points;
    detector(cartesian_mat, cv::noArray(), points, descriptors);

    std::vector<GlobalCartFeature> features;
    int n_points = points.size();
    if (n_points != descriptors.rows) {
        std::cerr << "ERROR: n_points != descriptors.rows" << std::endl;
        return features;
    } 
    cv::imshow("mat", cartesian_mat);
    features.resize(n_points);
    for (int i = 0; i < n_points; i++) {
        auto &kp = points[i];
        std::cout << kp.pt.x << "," << kp.pt.y << " ";
        features[i] = GlobalCartFeature{ kp.pt.y / scale, mapping.rangeEnd - kp.pt.x / scale, descriptors.rowRange(i,i+1)};
    }
    return features;
}
