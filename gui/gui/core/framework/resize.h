#ifndef RESIZE_H
#define RESIZE_H

#include <QGraphicsObject>
#include <QPen>
#include <QSizeF>

namespace cauv {
    namespace gui {

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

    } // namespace gui
} // namespace gui

#endif // RESIZE_H
