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

    connect(this, SIGNAL(imageUpdated(QImage)), this, SLOT(redraw(QImage)), Qt::BlockingQueuedConnection);
}

VideoScreen::~VideoScreen(){
    delete(ui);
}

void VideoScreen::setImage(QImage image){
    Q_EMIT imageUpdated(image);
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


    //m_updateMutex.lock();
    //if(m_image.height() > 0)
    //    aspect = m_image.width() / m_image.height();
    //m_updateMutex.unlock();

    int height = 300 * aspect;
    return QSize(300, height);
}

void VideoScreen::redraw(QImage image){
    m_image = image;
    repaint();
}


void VideoScreen::paintEvent(QPaintEvent * arg) {
    debug() << "Repaint started in VideoScreen";
    QImage copy = QImage(m_image);
    QPainter p(this);
    p.drawImage(rect(), copy);
    p.end();
    debug() << "Repaint finished in  VideoScreen";
}
