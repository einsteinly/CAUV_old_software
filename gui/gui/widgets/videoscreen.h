#ifndef VIDEOSCREEN_H
#define VIDEOSCREEN_H

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

    class VideoScreen : public QWidget
    {
        Q_OBJECT
    public:
        explicit VideoScreen(const QString name, QWidget *parent = 0);
        virtual ~VideoScreen();
        QSize sizeHint() const;

   Q_SIGNALS:
        void imageUpdated(QImage image);

    private:
        Ui::VideoScreen * ui;

    protected:
        QImage m_image;
        void paintEvent(QPaintEvent *);


    private Q_SLOTS:
        void redraw(QImage image);

    public Q_SLOTS:
        void setImage(Image &image);
        void setImage(QImage image);

        void setInfo(const QString text);
        void setInfo(const std::string text);

        void setName(const QString name);
        void setName(const std::string name);
    };
} // namespace cauv
#endif // VIDEOSCREEN_H
