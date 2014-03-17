#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include "sonar_ops.h"

namespace cauv {

class GlobalFeatureExtractor {
    public:
    virtual std::vector<GlobalCartFeature> extractFeatures(cv::Mat polarImage, PolarMapping &mapping) = 0;
};

}
