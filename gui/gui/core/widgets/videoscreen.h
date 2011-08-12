#ifndef VIDEOSCREEN_H
#define VIDEOSCREEN_H

#include <boost/thread.hpp>
#include <boost/scoped_ptr.hpp>

#include <QWidget>

#include "../model/variants.h"

class QSize;
class QImage;
class QString;

namespace Ui {
    class VideoScreen;
}

namespace cauv {

    namespace gui {

        class VideoScreen : public QWidget
        {
            Q_OBJECT
        public:
            explicit VideoScreen(const QString name, QWidget *parent = 0);
            virtual ~VideoScreen();
            //int heightForWidth( int w ) const;

        protected:
            boost::scoped_ptr<Image> m_new_image;
            boost::scoped_ptr<Image> m_current_image;

            void paintEvent(QPaintEvent *);

        public Q_SLOTS:
            void setImage(const image_variant_t &image);

            void setInfo(const QString text);
            void setInfo(const std::string text);

            void setName(const QString name);
            void setName(const std::string name);

        private:
            Ui::VideoScreen * ui;
            boost::mutex m_updateMutex;
            boost::scoped_ptr<QImage> m_qImage;
        };
    } // namespace gui
} // namespace cauv
#endif // VIDEOSCREEN_H
