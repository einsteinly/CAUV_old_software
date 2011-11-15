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

#ifndef __CAUV_FLUIDITY_IMGNODE_H__
#define __CAUV_FLUIDITY_IMGNODE_H__

#include "fluidity/fNode.h"

namespace cauv{
namespace gui{
namespace f{

class ImgNode: public FNode{
    public:
        ImgNode(Manager& m, node_id_t id, NodeType::e const& type);
        ImgNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p);

};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FLUIDITY_IMGNODE_H__
