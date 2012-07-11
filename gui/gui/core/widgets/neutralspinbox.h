/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef NEUTRALSPINBOX_H
#define NEUTRALSPINBOX_H

#include <QSpinBox>

#include <common/msg_classes/bounded_float.h>

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
            typedef cauv::BoundedFloat cauv__BoundedFloat;
            Q_PROPERTY(double value READ value WRITE setValue USER true)
            Q_PROPERTY(cauv__BoundedFloat boundedValue READ boundedValue WRITE setBoundedValue USER false)
            Q_PROPERTY(double neutral READ neutral WRITE setNeutral USER false)

            NeutralDoubleSpinBox(QWidget * parent = 0);

            double neutral() const;
            void setNeutral(double neutral);

            bool inverted() const;
            void setInverted(bool invert);

            void setBoundedValue(cauv::BoundedFloat value);
            cauv::BoundedFloat boundedValue() const;

            void paintEvent(QPaintEvent *);

        protected:
            double m_neutral;
            bool m_inverted;
        };

    } // namespace gui
} // namespace cauv

#endif // NEUTRALSPINBOX_H
