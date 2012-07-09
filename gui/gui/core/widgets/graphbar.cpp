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

#include <utility/pivot.h>
#include <debug/cauv_debug.h>

#include <limits>

#include "framework/style.h"

using namespace cauv;
using namespace cauv::gui;


GraphingSpinBox::GraphingSpinBox(QWidget * parent) :
    QSpinBox(parent) {
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<int>::max());
    this->setMinimum(std::numeric_limits<int>::min());
}

void GraphingSpinBox::setSampler(boost::shared_ptr<SampleQueue<QVariant> > sampler){
    disconnect(this, SLOT(repaint()));
    m_sampler = sampler;
    connect(m_sampler.get(), SIGNAL(timeout()), this, SLOT(repaint()));
}

boost::shared_ptr<SampleQueue<QVariant> > GraphingSpinBox::sampler() const {
    return m_sampler;
}

void GraphingSpinBox::paintEvent(QPaintEvent *)
 {
    StyleOptionGraphingSpinBox option;
    option.initFrom(this);

    option.maximum = maximum();
    option.minimum = minimum();
    option.samples = m_sampler->samples();

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}



GraphingDoubleSpinBox::GraphingDoubleSpinBox(QWidget * parent) :
    QDoubleSpinBox(parent) {
    this->setAlignment(Qt::AlignHCenter);
    this->setMaximum(std::numeric_limits<float>::max());
    this->setMinimum(std::numeric_limits<float>::min());
}

void GraphingDoubleSpinBox::setSampler(boost::shared_ptr<SampleQueue<QVariant> > sampler){
    disconnect(this, SLOT(repaint()));
    m_sampler = sampler;
    connect(m_sampler.get(), SIGNAL(timeout()), this, SLOT(repaint()));
}

boost::shared_ptr<SampleQueue<QVariant> > GraphingDoubleSpinBox::sampler() const {
    return m_sampler;
}


void GraphingDoubleSpinBox::paintEvent(QPaintEvent * )
 {
    StyleOptionNeutralSpinBox o;
    o.initFrom(this);

    o.level = pivot(minimum(), 0, maximum(), value());

    QPainter painter(this);
    style()->drawComplexControl(QStyle::CC_SpinBox, &o, &painter, this);


    painter.setOpacity(0.5);
    StyleOptionGraphingSpinBox option;
    option.initFrom(this);

    option.maximum = maximum();
    option.minimum = minimum();
    option.samples = m_sampler->samples();

    style()->drawComplexControl(QStyle::CC_SpinBox, &option, &painter, this);
}

