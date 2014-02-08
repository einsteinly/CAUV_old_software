
#include "corner_feature_detector.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <algorithm>

using namespace cauv;

std::vector<LocalPolarFeature>
cauv::CornerFeatureDetector::extractFeatures(cv::Mat polarImage) {
    std::vector<cv::KeyPoint> corners;
    std::vector<LocalPolarFeature> features;
    cv::FAST(polarImage, corners, 20, true);
    features.resize(corners.size());
    std::transform(corners.begin(), corners.end(), features.begin(),
                   [](cv::KeyPoint &kp) {
                        return LocalPolarFeature {kp.pt.x, kp.pt.y, 0};
                    });
    return features;
}
