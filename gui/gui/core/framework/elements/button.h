#ifndef __CAUV_ELEMENT_BUTTON_HEADER_H__
#define __CAUV_ELEMENT_BUTTON_HEADER_H__

#include <QGraphicsObject>

namespace cauv{
namespace gui{

struct NodeStyle;

class Button: public QGraphicsObject{
    Q_OBJECT
    public:
        Button(QRectF clip, QString base_fname, QGraphicsItem *parent=NULL);

    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget *w=0);
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
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_BUTTON_HEADER_H__


