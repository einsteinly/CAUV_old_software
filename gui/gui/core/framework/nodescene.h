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

#ifndef __CAUV_NODESCENE_H__
#define __CAUV_NODESCENE_H__

#include <QtGui>

#include <gui/core/nodedragging.h>

namespace cauv {
namespace gui {

class VanishingTextItem : public QGraphicsTextItem {

public:
    VanishingTextItem(QString const& text, float lod);
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    float m_lod;
};

class VanishingLineItem : public QGraphicsLineItem {

public:
    VanishingLineItem ( float lod, QGraphicsItem * parent = 0 );
    VanishingLineItem ( float lod, const QLineF & line, QGraphicsItem * parent = 0 );
    VanishingLineItem ( float lod, qreal x1, qreal y1, qreal x2, qreal y2, QGraphicsItem * parent = 0 );

    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

protected:
    float m_lod;
};


class NodeScene : public QGraphicsScene, public NodeDropListener {

public:
    typedef DropHandlerInterface<QGraphicsItem *> drop_handler_t;
    typedef boost::shared_ptr<drop_handler_t> drop_handler_ptr;

    NodeScene(QObject * parent = NULL);
    virtual ~NodeScene();

    // drop handlers
    virtual void registerDropHandler(drop_handler_ptr const& handler);
    virtual void removeDropHandler(drop_handler_ptr const& handler);

    // drop listener methods
    bool accepts(boost::shared_ptr<Node> const& node);
    virtual void onNodeDroppedAt(boost::shared_ptr<Node> const&, QPointF );

protected:
    QGraphicsItem * applyHandlers(boost::shared_ptr<Node> const& node); 

    std::vector<drop_handler_ptr> m_handlers;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_NODESCENE_H__
