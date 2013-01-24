/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ELEMENT_NODE_INPUT_H__
#define __CAUV_ELEMENT_NODE_INPUT_H__

#include <QGraphicsPathItem>
#include <QGraphicsLayoutItem>

#include "liquid/node.h"
#include "liquid/requiresCutout.h"

namespace liquid {
    struct NodeStyle;
}

namespace cauv{
namespace gui{

// !!! synchronise with pipeline
namespace NodeIOType {
enum e {
    Image, Parameter
};
}

class NodeInput: public QGraphicsPathItem,
                 public QGraphicsLayoutItem,
                 public liquid::RequiresCutout{
    public:
        NodeInput(liquid::NodeStyle const& style, NodeIOType::e const& type,
                  bool required, QGraphicsItem *parent=NULL);

        virtual QList<liquid::CutoutStyle> cutoutGeometry() const;
    
        virtual void setGeometry(QRectF const& rect);

    protected:
        virtual QSizeF sizeHint(Qt::SizeHint which,
                                const QSizeF&
                                constraint=QSizeF()) const;

    protected:
        NodeIOType::e m_type;
        bool m_required;
        liquid::NodeStyle const& m_style;
        
        // temp debug thing
        QRectF m_rect;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_NODE_INPUT_H__

