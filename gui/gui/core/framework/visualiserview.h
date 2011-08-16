#ifndef VISUALISERVIEW_H
#define VISUALISERVIEW_H

#include <QGraphicsView>
#include <QWheelEvent>

namespace cauv {
    namespace gui {

        class VisualiserView : public QGraphicsView
        {
        public:
            VisualiserView(QWidget * parent = NULL);
            virtual ~VisualiserView();

            virtual float scaleFactor();
            virtual void setScaleFactor(float scaleFactor);            

            void wheelEvent(QWheelEvent *event);

        protected:
            float m_scaleFactor;
            float m_scale;
        };

    } // namespace gui
} // namespace gui


#endif // VISUALISERVIEW_H
