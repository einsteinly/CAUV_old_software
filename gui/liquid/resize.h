/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
