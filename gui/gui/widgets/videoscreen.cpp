#include "videoscreen.h"
#include "ui_videoscreen.h"

#include <QPainter>
#include <QBrush>
#include <QColor>
#include <QPixmap>
#include <QImage>
#include <QSize>

#include <opencv/cv.h>

#include <debug/cauv_debug.h>

#include <generated/messages.h>

using namespace cauv;


VideoScreen::VideoScreen(const QString name, QWidget *parent) :
        QWidget(parent), m_new_image(0), m_current_image(0), ui(new Ui::VideoScreen())
{
    ui->setupUi(this);
    setName(name);
}

VideoScreen::~VideoScreen(){
    delete(ui);
}


void VideoScreen::setImage(const Image &img){

    // take a deep copy of the image before it's deleted
    // it will be used in a different thread so we can't
    // just keep a pointer
    if(m_updateMutex.try_lock()) {
        boost::scoped_ptr<Image> newImage(new Image(img));
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

int VideoScreen::heightForWidth( int w ) const {
    float aspect = 1;

    //m_updateMutex.lock();
    //if(m_image.height() > 0)
    //    aspect = m_image.width() / m_image.height();
    //m_updateMutex.unlock();

    std::cout <<"height for width " << w * aspect;

    return w * aspect;
}

void VideoScreen::paintEvent(QPaintEvent*) {

    m_updateMutex.lock();

    if(m_new_image.get()) {
        // replace the old image
        m_current_image.swap(m_new_image);
        m_new_image.reset();

        // convert the new one
        try {
            cv::Mat mat = m_current_image->cvMat();
            int channels = mat.channels();
            if(channels == 1) {
                cv::Mat result;
                cv::cvtColor(mat, result,CV_GRAY2RGB);
                mat = result;
            }
            else if(channels == 3) {
                cv::cvtColor(mat, mat, CV_BGR2RGB);
            }

            boost::scoped_ptr<QImage> replacement(new QImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_RGB888));
            m_qImage.swap(replacement);

        } catch (cv::Exception ex){
            error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__ << " " << ex.msg;
        }
    }
    m_updateMutex.unlock();


    QPainter p(this);
    if(m_qImage.get())
        p.drawImage(rect(), *m_qImage.get());
    p.end();
}
