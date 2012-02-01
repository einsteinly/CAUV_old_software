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

#include <QPaintEvent>
#include <QPainter>
#include <QLineEdit>

#include <common/cauv_utils.h>

#include <debug/cauv_debug.h>

#include <limits>

#include "style.h"

using namespace cauv;
using namespace cauv::gui;


NeutralSpinBox::NeutralSpinBox(QWidget * parent) : QSpinBox(parent), m_neutral(0){
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

void NeutralSpinBox::paintEvent(QPaintEvent * )
 {
    StyleOptionNeutralSpinBox option;
    option.initFrom(this);

    option.level = pivot(minimum(), neutral(), maximum(), value());

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}



NeutralDoubleSpinBox::NeutralDoubleSpinBox(QWidget * parent) : QDoubleSpinBox(parent), m_neutral(0){
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<float>::max());
    this->setMinimum(std::numeric_limits<float>::min());

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

void NeutralDoubleSpinBox::paintEvent(QPaintEvent * )
 {
    StyleOptionNeutralSpinBox option;
    option.initFrom(this);

    option.level = pivot(minimum(), neutral(), maximum(), value());

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}

