#ifndef __CAUV_ELEMENT_NODE_H__
#define __CAUV_ELEMENT_NODE_H__

#include <QGraphicsObject>
#include <QGraphicsPathItem>

namespace cauv{
namespace gui{

struct NodeStyle;

class Node: public QGraphicsObject{
    Q_OBJECT
    public:
        Node(NodeStyle const& style);
        
        //void populate(...);

    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget *w=0);
    
    protected Q_SLOTS:
        void updateLayout();

    protected:
        // ...
        //QVector<QGraphicsLineItem*> m_separators;
        
        QGraphicsPathItem* m_back;

        NodeStyle const& m_style;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_NODE_H__
