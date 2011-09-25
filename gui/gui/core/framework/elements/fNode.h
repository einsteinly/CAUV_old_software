#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <liquid/node.h>

#include "fluidity/managedElement.h"

namespace cauv{
namespace gui{

    class FNode: public liquid::LiquidNode,
             public ManagedElement{
    public:
        FNode(Manager& m, QGraphicsItem *parent=0);
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
