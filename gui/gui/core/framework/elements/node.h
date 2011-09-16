#ifndef __CAUV_ELEMENT_NODE_H__
#define __CAUV_ELEMENT_NODE_H__

#include <QObject>
#include <QGraphicsPathItem>

namespace cauv{
namespace gui{

struct NodeStyle;

class Node: public QObject,
            public QGraphicsPathItem{
    Q_OBJECT
    public:
        Node(NodeStyle const& style);
        
        //void populate(...);

    protected Q_SLOTS:
        void updateLayout();

    protected:
        // ...
        //QVector<QGraphicsLineItem*> m_separators;

        NodeStyle const& m_style;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_NODE_H__
