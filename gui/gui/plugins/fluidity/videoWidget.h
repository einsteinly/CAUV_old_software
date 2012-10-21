/* Copyright 2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_GUI_F_VIDEO_WIDGET_H__
#define __CAUV_GUI_F_VIDEO_WIDGET_H__

#include <QGraphicsWidget>

#include <boost/shared_ptr.hpp>

class QPixmap;

namespace cauv{

class GuiImageMessage;

namespace gui{
namespace f{

class VideoWidget: public QGraphicsWidget{
        Q_OBJECT
    public:
        VideoWidget(QGraphicsWidget* parent=0);

        void paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget=0);

    public Q_SLOTS:
        void displayImage(boost::shared_ptr<const GuiImageMessage>);

    private:
        boost::shared_ptr<const GuiImageMessage> m_image_msg;
        boost::shared_ptr<QPixmap> m_pixmap;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_F_VIDEO_WIDGET_H__
