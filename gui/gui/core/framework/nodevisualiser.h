#ifndef NODEVISUALISER_H
#define NODEVISUALISER_H

#include <QGraphicsView>
#include <QWheelEvent>

namespace cauv {
    namespace gui {

        class NodeVisualiser : public QGraphicsView
        {
        public:
            NodeVisualiser(QWidget * parent = NULL);

            virtual float scaleFactor();
            virtual void setScaleFactor(float scaleFactor);            

            void wheelEvent(QWheelEvent *event);

        protected:
            float m_scaleFactor;
            float m_scale;
        };

    } // namespace gui
} // namespace gui


#endif // NODEVISUALISER_H
