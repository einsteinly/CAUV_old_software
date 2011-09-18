#ifndef __CAUV_ELEMENT_BUTTON_HEADER_H__
#define __CAUV_ELEMENT_BUTTON_HEADER_H__

#include <QGraphicsWidget>
#include <QPen>
#include <QBrush>

namespace cauv{
namespace gui{

struct NodeStyle;

class Button: public QGraphicsWidget{
    Q_OBJECT
    public:
        Button(QRectF clip,
               QString base_fname,
               QAbstractGraphicsShapeItem *back_item=NULL,
               QGraphicsItem *parent=NULL);
        Button(QRectF clip,
               QGraphicsItem *default_item,
               QGraphicsItem *hover_item=NULL,
               QGraphicsItem *pressed_item=NULL,
               QAbstractGraphicsShapeItem *back_item=NULL,
               QGraphicsItem *parent=NULL);
        
        // sets background pen & brush if the background is set
        void setPen(QPen pen);
        void setBrush(QBrush pen);

    protected:
        virtual QSizeF sizeHint(Qt::SizeHint which, const QSizeF &constraint = QSizeF()) const;
        virtual QRectF boundingRect() const;
        virtual QPainterPath shape() const;

        virtual void hoverEnterEvent(QGraphicsSceneHoverEvent *event);
        virtual void hoverLeaveEvent(QGraphicsSceneHoverEvent *event);
        virtual void mousePressEvent(QGraphicsSceneMouseEvent *event);
        virtual void mouseMoveEvent(QGraphicsSceneMouseEvent *event);
        virtual void mouseReleaseEvent(QGraphicsSceneMouseEvent *event);

    Q_SIGNALS:
        void pressed();

    protected Q_SLOTS:
        void delayedRaise();

    protected:
        QGraphicsPixmapItem* loadPix(QString n);

        QRectF m_clip;

        QGraphicsItem* m_default;
        QGraphicsItem* m_hover;
        QGraphicsItem* m_pressed;

        QAbstractGraphicsShapeItem* m_background;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_BUTTON_HEADER_H__


