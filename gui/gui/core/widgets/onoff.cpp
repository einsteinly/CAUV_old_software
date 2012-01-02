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

#include "onoff.h"

#include <QPaintEvent>
#include <QPainter>

#include "style.h"

using namespace cauv;
using namespace cauv::gui;


OnOffSlider::OnOffSlider(QWidget * parent) :
    QCheckBox(parent), m_position(0), m_animation(this, "position") {
    m_animation.connect(&m_animation, SIGNAL(finished()), this, SIGNAL(switched()));
    this->connect(this, SIGNAL(switched()), this, SLOT(toggle()));
    m_animation.setDuration(100);
}

void OnOffSlider::mouseReleaseEvent(QMouseEvent *e) {
    bool checked = this->isChecked();
    QCheckBox::mouseReleaseEvent(e);
    switchTo(!checked);
}

void OnOffSlider::switchTo(bool state) {
    m_animation.stop();
    m_animation.setStartValue(position());
    m_animation.setEndValue(state ? 1.0 : 0.0);
    m_animation.start();
}

void OnOffSlider::setPosition(float position){
    m_position = position;
    update();
}

float OnOffSlider::position(){
    return m_position;
}

void OnOffSlider::setChecked(bool state){
    m_animation.stop();
    QCheckBox::setChecked(state);
    m_position = state ? 1 : 0;
}

void OnOffSlider::paintEvent(QPaintEvent * e)
 {
    StyleOptionOnOff option;
    option.initFrom(this);

    option.position = m_position;

    QPainter painter(this);
    style()->drawControl(QStyle::CE_CheckBox, &option, &painter, this);
}

