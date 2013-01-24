/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ONOFF_H
#define __CAUV_ONOFF_H

#include <QCheckBox>
#include <QPropertyAnimation>

#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

        class OnOffSlider : public QWidget {
            Q_OBJECT

        public:

            Q_PROPERTY(bool checked READ isChecked WRITE setChecked USER true)
            Q_PROPERTY(float position READ position WRITE setPosition USER false)

            OnOffSlider(QWidget * parent = 0);
            virtual ~OnOffSlider();

            void paintEvent(QPaintEvent *);

            void animateSwitch();

            void setAnimation(bool);

            void mouseReleaseEvent(QMouseEvent *e);
            void mouseMoveEvent(QMouseEvent *e);
            void mousePressEvent(QMouseEvent *e);

            bool isChecked();
            void setChecked(bool checked);

            void toggle();

        Q_SIGNALS:
            void switched();

        protected:
            float m_position;
            bool checked;
            QPropertyAnimation m_animation;

            float position();
            void setPosition(float position);
        };

    } // namespace gui
} // namespace cauv

#endif // __CAUV_ONOFF_H
