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

#include "neutralspinbox.h"


using namespace cauv;
using namespace cauv::gui;


NeutralSpinBox::NeutralSpinBox(QWidget * parent) : QSpinBox(parent), m_neutral(0){
    this->setAlignment(Qt::AlignHCenter);
}

int NeutralSpinBox::getNeutral() const {
    return m_neutral;
}

void NeutralSpinBox::setNeutral(int neutral){
    m_neutral = neutral;
}


BoundedFloatSpinBox::BoundedFloatSpinBox(QWidget * parent) : QDoubleSpinBox(parent), m_boundedValue(), m_neutral(0){
    this->setAlignment(Qt::AlignHCenter);
    info() << "BoundedFloatSpinBox()";
}

/*void BoundedFloatSpinBox::setValue(double value){
    info() << "setting value";
    info() << value;

    //QDoubleSpinBox::setValue(value.value);
    //setMinimum(value.min);
    //setMaximum(value.max);
    //setWrapping(value.type == BoundedFloatType::Wraps);
    //m_boundedValue = value;
}

double BoundedFloatSpinBox::value() const{
    info() << "reading value" << QDoubleSpinBox::value();
    //BoundedFloat r(m_boundedValue);
    //r.value = QDoubleSpinBox::value();
    //info() << r;
    return 2;//r;
    //return BoundedFloat(value(),minimum(),maximum(), wrapping() ? BoundedFloatType::Wraps : BoundedFloatType::Clamps);
}*/

double BoundedFloatSpinBox::neutral() const {
    return m_neutral;
}

void BoundedFloatSpinBox::setNeutral(double neutral){
    m_neutral = neutral;
}



