#pragma once

#include "local_feature_extractor.h"

namespace cauv {

class CornerFeatureDetector: public LocalFeatureExtractor {
    public:
    virtual std::vector<LocalPolarFeature> extractFeatures(cv::Mat polarImage);
};

}
