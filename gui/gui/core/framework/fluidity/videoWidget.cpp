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

#include "videoWidget.h"

#include <QImage>
#include <QPainter>
#include <QGraphicsScene>

#include <boost/make_shared.hpp>

#include <common/msg_classes/base_image.h>
#include <debug/cauv_debug.h>

#include <generated/types/GuiImageMessage.h>

using namespace cauv;
using namespace cauv::gui::f;

VideoWidget::VideoWidget(QGraphicsWidget* parent)
    : QGraphicsWidget(parent){
    qRegisterMetaType< boost::shared_ptr<const GuiImageMessage> >("boost::shared_ptr<const GuiImageMessage>");
    setMinimumSize(QSizeF(40,40));
    setPreferredSize(QSizeF(4000,400));
    setMaximumSize(QSizeF(10000,10000));
}

void VideoWidget::paint(QPainter* painter, const QStyleOptionGraphicsItem* option, QWidget* widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    if(m_qimg){
        debug(10) << "painting image"
                  << m_qimg->size().width() << "x" << m_qimg->size().height() << "->"
                  << boundingRect().width() << "x" << boundingRect().height();
        // Fixed aspect ratio...
        QRectF fixed_ratio;
        QRectF br = boundingRect();
        float img_w_over_h = float(m_qimg->size().width()) / m_qimg->size().height();
        if(img_w_over_h <= br.width() / br.height()){
            const float height = br.height();
            const float width = height * img_w_over_h;
            const float x_offset = 0.5 * (br.width() - width);
            fixed_ratio = QRectF(x_offset, 0, width, height);
        }else{
            const float width = br.width();
            const float height = width / img_w_over_h;
            const float y_offset = 0.5 * (br.height() - height);
            fixed_ratio = QRectF(0, y_offset, width, height);
        }
        painter->drawImage(fixed_ratio, *m_qimg);
    }
}

void VideoWidget::displayImage(boost::shared_ptr<const GuiImageMessage> p){
    m_image_msg = p;
    BaseImage t;
    m_image_msg->get_image_inplace(t);
    m_qimg = boost::make_shared<QImage>();
    m_qimg->loadFromData(&t.bytes()[0], t.bytes().size(), "jpg");
    scene()->invalidate(sceneBoundingRect());
}

