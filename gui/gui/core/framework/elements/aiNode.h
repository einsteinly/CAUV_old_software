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

#ifndef __CAUV_ELEMENT_AI_NODE_H__
#define __CAUV_ELEMENT_AI_NODE_H__

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>

#include <QBrush>
#include <QPen>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsItem>

#include <liquid/node.h>

#include "elements/arc.h"

namespace cauv {
    namespace gui {

        class ResizeHandle;
        class Button;

        class AINode : public liquid::LiquidNode, public ConnectableInterface
        {
            Q_OBJECT
            typedef liquid::LiquidNode base_t;
        public:
            AINode(QGraphicsItem *parent = 0);
            virtual ~AINode();

            virtual QGraphicsObject * asQGraphicsObject();

        public: // !!! TODO: protected
            virtual void setSize(QSizeF const&);

        signals:
            void boundriesChanged();
            void disconnected();

        };

    } // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_AI_NODE_H__
