#ifndef GRAPHICSWINDOW_H
#define GRAPHICSWINDOW_H

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>

#include <QBrush>
#include <QPen>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsItem>


namespace cauv {
    namespace gui {


        class GraphicsWindowButton : public QGraphicsWidget {
            Q_OBJECT

        public:

            GraphicsWindowButton(QGraphicsItem * item, qreal width = 20, qreal height = 20);
            virtual ~GraphicsWindowButton();

            virtual QSizeF sizeHint(Qt::SizeHint which, const QSizeF &constraint = QSizeF()) const;
            void setPen(QPen const& pen);

            void hoverEnterEvent(QGraphicsSceneHoverEvent *);
            void hoverLeaveEvent(QGraphicsSceneHoverEvent *);
            void mousePressEvent(QGraphicsSceneMouseEvent *);
            void mouseReleaseEvent(QGraphicsSceneMouseEvent *);

        Q_SIGNALS:
            void pressed();

        protected:
            QGraphicsItem * m_item;
            QGraphicsEllipseItem * m_background;
        };


        class GraphicsWindow : public QGraphicsObject
        {
            Q_OBJECT
        public:
            GraphicsWindow(QGraphicsItem *parent = 0);
            virtual ~GraphicsWindow();
            QRectF boundingRect() const;
            virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);

            virtual QSizeF size() const;
            virtual void setSize(QSizeF);

            virtual qreal cornerRadius() const;
            virtual void setCornerRadius(qreal);

            virtual QPen backgroundPen() const;
            virtual void setBackgroundPen(QPen);

            virtual QBrush backgroundBrush() const;
            virtual void setBackgroundBrush(QBrush);

            virtual void addButton(GraphicsWindowButton * button);

            virtual void setClosable(bool);

        protected:
            QSizeF m_size;
            qreal m_cornerRadius;
            QPen m_backgroundPen;
            QBrush m_backgroundBrush;

            QGraphicsWidget * m_buttonsWidget;
            QGraphicsLinearLayout * m_layout;
            GraphicsWindowButton * m_closeButton;

        signals:

            void closed(GraphicsWindow *);
            void moved(QPointF);

        public slots:
            void close();

        };

    } // namespace gui
} // namespace gui

#endif // GRAPHICSWINDOW_H
