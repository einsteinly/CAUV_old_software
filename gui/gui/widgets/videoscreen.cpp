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

void VideoScreen::setImage(QPixmap &pixmap){
    m_pixmap = pixmap;
}

void VideoScreen::setImage(QImage &image){
    QPixmap pixmap = QPixmap::fromImage(image);
    setImage(pixmap);
}

void VideoScreen::setImage(Image &img){
    try {
        cv::Mat mat_rgb;
        cv::cvtColor(img.cvMat(), mat_rgb, CV_BGR2RGB);
        QImage qImage = QImage((const unsigned char*)(mat_rgb.data), mat_rgb.cols, mat_rgb.rows, QImage::Format_RGB888);
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
    if(m_pixmap.height() > 0)
        aspect = m_pixmap.width() / m_pixmap.height();
    int height = 300 * aspect;
    return QSize(300, height);
}

void VideoScreen::paintEvent(QPaintEvent * arg) {
    QPainter p(this);
    p.fillRect(rect(), QBrush(QColor::fromRgb(0,0,0)));
    p.drawPixmap(contentsRect(), m_pixmap);
    p.end();
    QWidget::paintEvent(arg);
}
