/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "../../nodeFactory.h"

#include "sonarInputNode.h"
#include "sonarShadowFilterNode.h"

namespace cauv{
namespace imgproc{

// Register node types (actually definitions of static data members)
DEFINE_NFR(SonarInputNode, NodeType::SonarInput);
DEFINE_NFR(SonarShadowFilterNode, NodeType::SonarShadowFilter);

} // namespace imgproc
} // namespace cauv

