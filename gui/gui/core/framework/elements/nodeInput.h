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

