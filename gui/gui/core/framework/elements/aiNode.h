#ifndef __CAUV_ELEMENT_AI_NODE_H__
#define __CAUV_ELEMENT_AI_NODE_H__

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>

#include <QBrush>
#include <QPen>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsItem>

#include "elements/graphicsWindow.h"
#include "elements/arc.h"

namespace cauv {
    namespace gui {

        class ResizeHandle;
        class Button;

        class AINode : public GraphicsWindow, public ConnectableInterface
        {
            Q_OBJECT
            typedef GraphicsWindow base_t;
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
