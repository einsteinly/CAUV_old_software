#ifndef __CAUV_ELEMENT_GRAPHICS_WINDOW_H__
#define __CAUV_ELEMENT_GRAPHICS_WINDOW_H__

#include <QGraphicsObject>

#include "fluidity/managedElement.h"

class QGraphicsLayoutItem;
class QGraphicsLinearLayout;

namespace cauv{
namespace gui{

struct NodeStyle;
class NodeHeader;
class Button;
class ResizeHandle;

class GraphicsWindow: public QGraphicsObject{
    Q_OBJECT
    public:
        GraphicsWindow(NodeStyle const& style, QGraphicsItem *parent=0);

    Q_SIGNALS:
        void closed(GraphicsWindow *);
    public Q_SLOTS:
        void close();

    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter*, const QStyleOptionGraphicsItem*, QWidget *w=0);
        
    public: // !!! TODO: protected!
        virtual QSizeF size() const;
        virtual void setSize(QSizeF const&);
    
        virtual void addButton(Button * button);
        virtual void addItem(QGraphicsLayoutItem *item);
    
        virtual void setClosable(bool);
        virtual void setResizable(bool);

        virtual void layoutChanged();

    protected Q_SLOTS:
        void updateLayout();
        void resized();

    protected:
        QSizeF m_size;

        NodeHeader            *m_header;
        QGraphicsWidget       *m_buttonsWidget;
        ResizeHandle          *m_resizeHandle;
        QGraphicsWidget       *m_contentWidget;
        QGraphicsLinearLayout *m_contentLayout;
        QGraphicsPathItem     *m_back;        
        // ...
        //QVector<QGraphicsLineItem*> m_separators;

        NodeStyle const& m_style;

};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_GRAPHICS_WINDOW_H__
