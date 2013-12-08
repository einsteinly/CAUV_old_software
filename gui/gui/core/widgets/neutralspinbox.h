/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef NEUTRALSPINBOX_H
#define NEUTRALSPINBOX_H

#include <QSpinBox>

#warning TODO

namespace cauv {
    namespace gui {

        class NeutralSpinBox : public QSpinBox {
            Q_OBJECT

        public:

            Q_PROPERTY(int value READ value WRITE setValue USER true)
            Q_PROPERTY(int neutral READ neutral WRITE setNeutral USER false)

            NeutralSpinBox(QWidget * parent = 0);

            int neutral() const;
            void setNeutral(int neutral);

            bool inverted() const;
            void setInverted(bool invert);

            void paintEvent(QPaintEvent *);

        protected:
            int m_neutral;
            bool m_inverted;
        };



        class NeutralDoubleSpinBox : public QDoubleSpinBox {
            Q_OBJECT

        public:
            //typedef cauv::BoundedFloat cauv__BoundedFloat;
            Q_PROPERTY(double value READ value WRITE setValue USER true)
            //Q_PROPERTY(cauv__BoundedFloat boundedValue READ boundedValue WRITE setBoundedValue USER false)
            Q_PROPERTY(double neutral READ neutral WRITE setNeutral USER false)

            NeutralDoubleSpinBox(QWidget * parent = 0);

            double neutral() const;
            void setNeutral(double neutral);

            bool inverted() const;
            void setInverted(bool invert);

            //void setBoundedValue(cauv::BoundedFloat value);
            //cauv::BoundedFloat boundedValue() const;

            void paintEvent(QPaintEvent *);

        protected:
            double m_neutral;
            bool m_inverted;
        };

    } // namespace gui
} // namespace cauv

#endif // NEUTRALSPINBOX_H
