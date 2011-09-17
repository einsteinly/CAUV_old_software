#ifndef __CAUV_ELEMENT_NODE_HEADER_H__
#define __CAUV_ELEMENT_NODE_HEADER_H__

#include <QGraphicsObject>

class QGraphicsPathItem;
class QGraphicsSimpleTextItem;

namespace cauv{
namespace gui{

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
        void closePressed();
        void collapsePressed();
        void execPressed();
        void duplicatePressed();

    public Q_SLOTS:
        // ...
        void setWidth(qreal w);

    protected:
        NodeStyle const& m_style;
        qreal m_width;

        Button *m_closebutton;
        Button *m_collapsebutton;
        Button *m_execbutton;
        Button *m_dupbutton;
        
        QGraphicsPathItem* m_overlay_back;
        QGraphicsSimpleTextItem* m_title;
        QGraphicsSimpleTextItem* m_info_text;

};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_NODE_HEADER_H__

