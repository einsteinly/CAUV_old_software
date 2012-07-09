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

#ifndef __LIQUID_RESIZE_H__
#define __LIQUID_RESIZE_H__

#include <QGraphicsObject>
#include <QPen>
#include <QSizeF>

namespace liquid {

class ResizeHandle : public QGraphicsObject {
    Q_OBJECT
public:
    ResizeHandle(QGraphicsObject *parent=0);

    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem * option, QWidget *);

    QPen pen() const;
    void setPen(QPen const& pen);

    QSizeF newSize() const;
    QSizeF size() const;
    void setSize(QSizeF const& s);

protected:
    QSizeF m_size;
    QPen m_pen;
};

} // namespace liquid

#endif // __LIQUID_RESIZE_H__
