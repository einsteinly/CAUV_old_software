#pragma once

#include "global_feature_extractor.h"

namespace cauv {

class SURFExtractor: public GlobalFeatureExtractor {
    public:
    virtual std::vector<GlobalCartFeature> extractFeatures(cv::Mat polarImage, PolarMapping &mapping);
    private:
};

}
