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
        QWidget(parent), ui(new Ui::VideoScreen())
{
    ui->setupUi(this);
    setName(name);
}

VideoScreen::~VideoScreen(){
    delete(ui);
}

void VideoScreen::setImage(QImage &image){
    m_updateMutex.lock();
    m_image = image.copy();
    m_updateMutex.unlock();
    update();
}

void VideoScreen::setImage(Image &img){
    try {
        debug() << "Image passed to VideoScreen";
        cv::Mat mat_rgb;
        cv::cvtColor(img.cvMat(), mat_rgb, CV_BGR2RGB);
        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols, mat_rgb.rows, QImage::Format_RGB888);
        debug() << "QImage created in VideoScreen";
        setImage(qImage);
        debug() << "QImage set";
    } catch (cv::Exception ex){
        error() << "cv::Exception thrown in " << __FILE__ << "on line" << __LINE__ << " " << ex.msg;
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

QSize VideoScreen::sizeHint() const{
    float aspect = 1;

    if(m_image.height() > 0)
        aspect = m_image.width() / m_image.height();

    int height = 300 * aspect;
    return QSize(300, height);
}

void VideoScreen::paintEvent(QPaintEvent * arg) {
    debug() << "Repaint started in VideoScreen";
    QPainter p(this);
    m_updateMutex.lock();
    p.drawImage(rect(), m_image);
    m_updateMutex.unlock();
    p.end();
    debug() << "Repaint finished in  VideoScreen";
}
