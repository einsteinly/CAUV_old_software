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

#include "videoscreen.h"
#include "ui_videoscreen.h"

#include <QPainter>
#include <QBrush>
#include <QColor>
#include <QPixmap>
#include <QImage>
#include <QSize>

#include <debug/cauv_debug.h>
#include <common/msg_classes/image.h>

using namespace cauv;
using namespace cauv::gui;

VideoScreen::VideoScreen(const QString name, QWidget *parent) :
        QWidget(parent), m_new_image(0), m_current_image(0), ui(new Ui::VideoScreen())
{
    ui->setupUi(this);
    setName(name);
}

VideoScreen::~VideoScreen(){
    debug(5) << __FUNCTION__;
    delete(ui);
}


void VideoScreen::setImage(const image_t &img){

    // take a deep copy of the image before it's deleted
    // it will be used in a different thread so we can't
    // just keep a pointer
    if(m_updateMutex.try_lock()) {
        boost::scoped_ptr<BaseImage> newImage(new BaseImage(*(img.get())));
        m_new_image.swap(newImage);

        update();

        m_updateMutex.unlock();
    }
}

void VideoScreen::setInfo(const QString info){
    ui->feedInfo->setText(info);
}

void VideoScreen::setInfo(const std::string info){
    VideoScreen::setInfo(QString::fromStdString(info));
}

void VideoScreen::setName(const QString name){
    ui->feedName->setText(name);
    setWindowTitle(name);
}

void VideoScreen::setName(const std::string name){
    VideoScreen::setName(QString::fromStdString(name));
}

/*
int VideoScreen::heightForWidth( int w ) const {
    float aspect = 1;

    //m_updateMutex.lock();
    //if(m_image.height() > 0)
    //    aspect = m_image.width() / m_image.height();
    //m_updateMutex.unlock();

    std::cout <<"height for width " << w * aspect;

    return w * aspect;
}
*/

void VideoScreen::paintEvent(QPaintEvent*) {

    // This conversion REALLY shouldn't be done here it slows down the GUI
    // thread causing a laggy interface if there's a lot of imcoming images
    //
    // Possible Solution: Have a dedicated conversion thread that takes the
    // latest image read by the messaging thread, converts it and queues a GUI
    // update in the GUI thread.
    //
    m_updateMutex.lock();

    if(m_new_image.get()) {
        m_qImage->loadFromData(&(m_new_image->bytes()[0]), m_new_image->bytes().size(), "jpg");
        m_new_image.reset();
        m_updateMutex.unlock();
    }

    QPainter p(this);
    if(m_qImage.get()) {
        p.drawImage(rect(), *m_qImage.get());
    }
    p.end();
}
