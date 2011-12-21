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

#include "graphbar.h"

#include <QPaintEvent>
#include <QPainter>

#include <common/cauv_utils.h>

#include <debug/cauv_debug.h>

#include <limits>

#include "style.h"

using namespace cauv;
using namespace cauv::gui;


GraphingSpinBox::GraphingSpinBox(QWidget * parent) :
    QSpinBox(parent), m_samples(this){
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<int>::max());
    this->setMinimum(std::numeric_limits<int>::min());
    m_samples.connect(&m_samples, SIGNAL(timeout()), this, SLOT(repaint()));
}


QList<int> GraphingSpinBox::values() const {
    return m_samples;
}

void GraphingSpinBox::paintEvent(QPaintEvent * e)
 {
    StyleOptionGraphingWidget option;
    option.initFrom(this);

    option.maximum = maximum();
    option.minimum = minimum();
    option.samples = values();

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}



GraphingDoubleSpinBox::GraphingDoubleSpinBox(QWidget * parent) :
    QDoubleSpinBox(parent), m_samples(this) {
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<float>::max());
    this->setMinimum(std::numeric_limits<float>::min());
    m_samples.connect(&m_samples, SIGNAL(timeout()), this, SLOT(repaint()));
}

QList<double> GraphingDoubleSpinBox::values() const {
    return m_samples;
}


void GraphingDoubleSpinBox::paintEvent(QPaintEvent * e)
 {
    StyleOptionGraphingWidget option;
    option.initFrom(this);

    //!!! HACK. this needs doing a different way
    // put the values into some sensible integer representation


    QList<int> asInts;
    float scalar = 1000 / (maximum() - minimum());
    foreach(double sample, (QList<double>) m_samples){
        asInts.append(sample*scalar);
    }

    option.maximum = maximum() * scalar;
    option.minimum = minimum() * scalar;
    option.samples = asInts;

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}

