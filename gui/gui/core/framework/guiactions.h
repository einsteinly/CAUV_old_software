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

#ifndef GUIACTIONS_H
#define GUIACTIONS_H

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <debug/cauv_debug.h>

class QGraphicsScene;

namespace liquid {
    class LiquidView;
}

namespace cauv {

    class CauvNode;

    namespace gui {

        class Vehicle;
        class NodePicker;
        class CauvMainWindow;
        class NodeScene;
        class VehicleItemModel;

        struct GuiActions {
            // weak ptrs to window (which is actually the same as node)
            // breaks the cycle of window -> actions -> window
            boost::weak_ptr<CauvMainWindow> window;
            boost::weak_ptr<CauvNode> node;

            // models
            boost::shared_ptr<Vehicle> auv;
            boost::shared_ptr<VehicleItemModel> root;
            boost::shared_ptr<NodeScene> scene;

            // gui elements
            liquid::LiquidView * view;
            NodePicker * nodes;

            ~GuiActions() {
                debug(2) << "~GuiActions()";
            }
        };
    } //namespace gui
} // namespace cauv


#endif // GUIACTIONS_H
