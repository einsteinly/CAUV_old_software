#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include "graphicsWindow.h"

namespace cauv{
namespace gui{

class FNode: public GraphicsWindow,
             public ManagedElement{
    public:
        FNode(Manager& m, QGraphicsItem *parent=0);

    protected:
        virtual void layoutChanged();
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
