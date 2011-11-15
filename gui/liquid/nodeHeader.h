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

#ifndef __LIQUID_NODE_HEADER_H__
#define __LIQUID_NODE_HEADER_H__

#include <QGraphicsObject>
#include <QMap>

class QGraphicsPathItem;
class QGraphicsSimpleTextItem;

namespace liquid{

struct NodeStyle;
class Button;

class NodeHeader: public QGraphicsObject{
    Q_OBJECT
    public:
        NodeHeader(NodeStyle const& style, QGraphicsObject *parent=0);


    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget *w=0);
    
        virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
        virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);

    Q_SIGNALS:
        /* none */

    public Q_SLOTS:
        void setTitle(QString title);
        void setInfo(QString info);
        void addButton(QString name, Button *button);
        Button* getButton(QString name);
        void setWidth(qreal w);

    protected:
        NodeStyle const& m_style;
        qreal m_width;
        
        QMap<QString,Button*> m_button_lookup;
        QList<Button*> m_buttons;

        QGraphicsPathItem* m_overlay_back;
        QGraphicsSimpleTextItem* m_title;
        QGraphicsSimpleTextItem* m_info_text;

};

} // namespace liquid

#endif // ndef __LIQUID_NODE_HEADER_H__

