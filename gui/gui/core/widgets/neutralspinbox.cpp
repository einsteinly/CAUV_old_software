/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "neutralspinbox.h"

#include <QPaintEvent>
#include <QPainter>
#include <QLineEdit>
#include <QMetaProperty>

#include <utility/pivot.h>
#include <debug/cauv_debug.h>

#include <limits>

#include "styles/style.h"

using namespace cauv;
using namespace cauv::gui;


NeutralSpinBox::NeutralSpinBox(QWidget * parent) : QSpinBox(parent), m_neutral(0), m_inverted(false){
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<int>::max());
    this->setMinimum(std::numeric_limits<int>::min());

    QPalette pal = this->palette();
    pal.setColor(QPalette::Base, Qt::transparent);
    this->setPalette(pal);
}

int NeutralSpinBox::neutral() const {
    return m_neutral;
}

void NeutralSpinBox::setNeutral(int neutral){
    m_neutral = neutral;
}

bool NeutralSpinBox::inverted() const {
    return m_inverted;
}

void NeutralSpinBox::setInverted(bool invert){
    m_inverted = invert;
}

void NeutralSpinBox::paintEvent(QPaintEvent * )
 {
    StyleOptionNeutralSpinBox option;
    option.initFrom(this);

    option.level = pivot((double)minimum(), (double)neutral(), (double)maximum(), (double)value());
    option.invertColours = inverted();

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}



NeutralDoubleSpinBox::NeutralDoubleSpinBox(QWidget * parent) : QDoubleSpinBox(parent), m_neutral(0), m_inverted(false){
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<float>::max());
    this->setMinimum(-std::numeric_limits<float>::max());

    QPalette pal = this->palette();
    pal.setColor(QPalette::Base, Qt::transparent);
    this->setPalette(pal);

}

double NeutralDoubleSpinBox::neutral() const {
    return m_neutral;
}

void NeutralDoubleSpinBox::setNeutral(double neutral){
    m_neutral = neutral;
}

bool NeutralDoubleSpinBox::inverted() const {
    return m_inverted;
}

void NeutralDoubleSpinBox::setInverted(bool invert){
    m_inverted = invert;
}

/*
void NeutralDoubleSpinBox::setBoundedValue(BoundedFloat value){
    info() << "set bounded value " << value;
    setMaximum(value.max);
    setMinimum(value.min);
    setValue(value.value);
    setWrapping(value.type == BoundedFloatType::Wraps);
}*/

/*
BoundedFloat NeutralDoubleSpinBox::boundedValue() const{
    BoundedFloat bf(value(), minimum(), maximum(),
                    wrapping()?BoundedFloatType::Wraps:BoundedFloatType::Clamps);
    return bf;
}*/

void NeutralDoubleSpinBox::paintEvent(QPaintEvent * )
 {
    StyleOptionNeutralSpinBox option;
    option.initFrom(this);

    option.level = pivot(minimum(), neutral(), maximum(), value());
    option.invertColours = inverted();

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}

