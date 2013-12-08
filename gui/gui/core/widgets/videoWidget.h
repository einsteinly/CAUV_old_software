/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_VIDEO_WIDGET_H__
#define __CAUV_GUI_VIDEO_WIDGET_H__

#include <QGraphicsWidget>

#include <boost/shared_ptr.hpp>

class QPixmap;

namespace cauv{

class GuiImageMessage;

namespace gui{

class VideoWidget: public QGraphicsWidget{
        Q_OBJECT
    public:
        VideoWidget(QGraphicsWidget* parent=0);

        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget=0);

    public Q_SLOTS:
        void displayImage(boost::shared_ptr<const GuiImageMessage>);

    private:
        //boost::shared_ptr<const GuiImageMessage> m_image_msg;
        boost::shared_ptr<QPixmap> m_pixmap;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_VIDEO_WIDGET_H__
