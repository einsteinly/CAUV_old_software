#ifndef GRAPHICSWINDOW_H
#define GRAPHICSWINDOW_H

#include <QGraphicsObject>

#include <boost/shared_ptr.hpp>

#include <QBrush>
#include <QPen>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QGraphicsItem>

#include "elements/arc.h"

namespace cauv {
    namespace gui {

        class ResizeHandle;
        class Button;

        class GraphicsWindow : public QGraphicsObject, public ConnectableInterface
        {
            Q_OBJECT
        public:
            GraphicsWindow(QGraphicsItem *parent = 0);
            virtual ~GraphicsWindow();
            QRectF boundingRect() const;
            virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget = 0);

            virtual QSizeF size() const;
            virtual void setSize(QSizeF const&);

            virtual qreal cornerRadius() const;
            virtual void setCornerRadius(qreal);

            virtual QPen backgroundPen() const;
            virtual void setBackgroundPen(QPen const&);

            virtual QBrush backgroundBrush() const;
            virtual void setBackgroundBrush(QBrush const&);

            virtual void addButton(Button * button);
            virtual void addItem(QGraphicsLayoutItem* item);

            virtual void setClosable(bool);
            virtual void setResizable(bool);

            virtual QGraphicsObject * asQGraphicsObject();

        protected:
            QSizeF m_size;
            qreal m_cornerRadius;
            QPen m_backgroundPen;
            QBrush m_backgroundBrush;

            QGraphicsWidget * m_buttonsWidget;
            QGraphicsLinearLayout * m_layout;
            Button * m_closeButton;
            ResizeHandle * m_resizeHandle;

            QGraphicsWidget * m_contentWidget;
            QGraphicsLinearLayout * m_contentLayout;


        signals:

            void closed(GraphicsWindow *);
            void boundriesChanged();
            void disconnected();

        public slots:
            void close();
            void resized();

        };

    } // namespace gui
} // namespace gui

#endif // GRAPHICSWINDOW_H
