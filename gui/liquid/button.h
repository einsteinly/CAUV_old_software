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

#ifndef __LIQUID_BUTTON_H__
#define __LIQUID_BUTTON_H__

#include <QGraphicsWidget>
#include <QPen>
#include <QBrush>

namespace liquid{

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
        void commonInit();
        virtual ~Button(){ }
        
        // sets background pen & brush if the background is set
        void setPen(QPen pen);
        void setBrush(QBrush pen);
        
        virtual QRectF boundingRect() const;
        virtual QPainterPath shape() const;

    protected:
        virtual QSizeF sizeHint(Qt::SizeHint which, const QSizeF &constraint = QSizeF()) const;

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

} // namespace liquid

#endif // ndef __LIQUID_BUTTON_H__


