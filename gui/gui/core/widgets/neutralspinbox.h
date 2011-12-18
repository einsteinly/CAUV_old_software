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

#include <common/bounded_float.h>


#include <debug/cauv_debug.h>

namespace cauv {
    namespace gui {

        class NeutralSpinBox : public QSpinBox {
            Q_OBJECT

        public:

            Q_PROPERTY(int neutral READ getNeutral WRITE setNeutral USER false)
            Q_PROPERTY(int value READ value WRITE setValue USER true)

            NeutralSpinBox(QWidget * parent = 0);

            int getNeutral() const;
            void setNeutral(int neutral);

        protected:
            int m_neutral;
        };



        class BoundedFloatSpinBox : public QDoubleSpinBox {
            Q_OBJECT

        public:
            //Q_PROPERTY(BoundedFloat value READ value WRITE setValue USER true)
            Q_PROPERTY(double value READ value WRITE setValue USER false)
            Q_PROPERTY(BoundedFloat boundedValue READ value2 WRITE setValue2 USER true)
            Q_PROPERTY(double neutral READ neutral WRITE setNeutral USER false)

            BoundedFloatSpinBox(QWidget * parent = 0);

            void setValue2(BoundedFloat value){
                info() << "setValue()";
                QDoubleSpinBox::setValue(value.value);
            }

            BoundedFloat value2() const{
                info() << "value()";
                //return QDoubleSpinBox::value();
                return BoundedFloat(QDoubleSpinBox::value(),2,3,BoundedFloatType::Clamps);
            }

            double neutral() const;
            void setNeutral(double neutral);

        protected:
            BoundedFloat m_boundedValue;
            double m_neutral;
        };

    } // namespace gui
} // namespace cauv

#endif // NEUTRALSPINBOX_H
