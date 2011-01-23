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
        cv::Mat mat = img.cvMat();

        int channels = mat.channels();

        if(channels == 1) {
            cv::Mat result;
            cv::cvtColor(mat, result,CV_GRAY2RGB);
            mat = result;
        }
        else if(channels == 3) {
            cv::cvtColor(mat, mat, CV_BGR2RGB);
        }

        QImage qImage((const unsigned char*)(mat.data), mat.cols, mat.rows, QImage::Format_RGB888);

        setImage(qImage);
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

    // can't lock mutex here as its const
    //if(m_image.height() > 0)
    //    aspect = m_image.width() / m_image.height();

    int height = 300 * aspect;
    return QSize(300, height);
}

void VideoScreen::paintEvent(QPaintEvent*) {
    m_updateMutex.lock();
    QPainter p(this);
    p.drawImage(rect(), m_image);
    p.end();
    m_updateMutex.unlock();
}
