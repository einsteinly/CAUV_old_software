#ifndef __CAUV_FLUIDITY_IMGNODE_H__
#define __CAUV_FLUIDITY_IMGNODE_H__

#include "fluidity/fNode.h"

namespace cauv{
namespace gui{
namespace f{

class ImgNode: public FNode{
    public:
        ImgNode(Manager& m, node_id_t id);
        ImgNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p);

};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_FLUIDITY_IMGNODE_H__
