#include "../nodeFactory.h"

#include "resizeNode.h"
#include "levelsNode.h"
#include "fastCornersNode.h"
#include "drawKeyPointsNode.h"
#include "gaussianBlurNode.h"
#include "fastMedianNode.h"
#include "sonarSLAMNode.h"
#include "bearingRangeToXYNode.h"

namespace cauv{
namespace imgproc{

DEFINE_NFR(ResizeNode, NodeType::Resize);
DEFINE_NFR(LevelsNode, NodeType::Levels);
DEFINE_NFR(FASTCornersNode, NodeType::FASTCorners);
DEFINE_NFR(DrawKeyPointsNode, NodeType::DrawKeyPoints);
DEFINE_NFR(GaussianBlurNode, NodeType::GaussianBlur);
DEFINE_NFR(FastMedianNode, NodeType::FastMedian);
DEFINE_NFR(SonarSLAMNode, NodeType::SonarSLAM);
DEFINE_NFR(BearingRangeToXYNode, NodeType::BearingRangeToXY);

} // namespace imgproc
} // namespace cauv


