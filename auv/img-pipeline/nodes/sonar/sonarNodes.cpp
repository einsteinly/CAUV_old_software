/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "../../nodeFactory.h"

#include "sonarInputNode.h"
#include "sonarShadowFilterNode.h"
#include "polarImageToXYNode.h"
#include "sonarImageEdgeNode.h"
#include "learnedKeyPointsNode.h"
#include "bearingRangeCropNode.h"

namespace cauv{
namespace imgproc{

// Register node types (actually definitions of static data members)
DEFINE_NFR(SonarInputNode, NodeType::SonarInput);
DEFINE_NFR(SonarShadowFilterNode, NodeType::SonarShadowFilter);
DEFINE_NFR(PolarImageToXYNode, NodeType::PolarImageToXY);
DEFINE_NFR(SonarImageEdgeNode, NodeType::SonarImageEdge);
DEFINE_NFR(LearnedKeyPointsNode, NodeType::LearnedKeyPoints);
DEFINE_NFR(BearingRangeCropNode, NodeType::BearingRangeCrop);

} // namespace imgproc
} // namespace cauv

