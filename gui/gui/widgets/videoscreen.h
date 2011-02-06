#ifndef VIDEOSCREEN_H
#define VIDEOSCREEN_H

#include <boost/signals/trackable.hpp>

#include <QWidget>
#include <QMutex>

class QPixmap;
class QSize;
class QString;

namespace Ui {
    class VideoScreen;
}

namespace cauv {
    class Image;

    class VideoScreen : public QWidget, public boost::signals::trackable
    {
        Q_OBJECT
    public:
        explicit VideoScreen(const QString name, QWidget *parent = 0);
        virtual ~VideoScreen();
        QSize sizeHint() const;

    private:
        Ui::VideoScreen * ui;

    protected:
        QMutex m_updateMutex;
        QImage m_image;
        void paintEvent(QPaintEvent *);

    public Q_SLOTS:
        void setImage(Image &image);
        void setImage(QImage &image);

        void setInfo(const QString text);
        void setInfo(const std::string text);

        void setName(const QString name);
        void setName(const std::string name);
    };
} // namespace cauv
#endif // VIDEOSCREEN_H
