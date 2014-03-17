#pragma once

#include <vector>
#include <opencv2/core/core.hpp>
#include "sonar_ops.h"

namespace cauv {

class LocalFeatureExtractor {
    public:
    virtual std::vector<LocalPolarFeature> extractFeatures(cv::Mat polarImage) = 0;
};

}
