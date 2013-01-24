/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "onoff.h"

#include <QPaintEvent>
#include <QPainter>

#include <QDebug>

#include "styles/style.h"

using namespace cauv;
using namespace cauv::gui;


OnOffSlider::OnOffSlider(QWidget * parent) :
    QWidget(parent), m_position(0), m_animation(this, "position") {
    m_animation.setDuration(0);
    setFocusPolicy(Qt::StrongFocus);
    connect(&m_animation, SIGNAL(finished()), this, SIGNAL(switched()));
}

OnOffSlider::~OnOffSlider() {
}


void OnOffSlider::animateSwitch() {
    m_animation.stop();
    m_animation.setStartValue(position());
    m_animation.setEndValue(isChecked() ? 1.0 : 0.0);
    m_animation.start();
}

void OnOffSlider::setPosition(float position){
    m_position = position;
    update();
}

float OnOffSlider::position(){
    return m_position;
}

void OnOffSlider::setChecked(bool c){
    checked = c;
    setPosition(c?1:0);
}

bool OnOffSlider::isChecked(){
    return checked;
}

void OnOffSlider::setAnimation(bool animates){
    if(animates){
        m_animation.setDuration(100);
    } else m_animation.setDuration(0);
}

void OnOffSlider::toggle(){
    checked = !checked;
    animateSwitch();
}

void OnOffSlider::mouseReleaseEvent(QMouseEvent *e) {
    e->accept();
}

void OnOffSlider::mouseMoveEvent(QMouseEvent *e){
    e->accept();
}

void OnOffSlider::mousePressEvent(QMouseEvent *e){
    toggle();
    e->accept();
}

void OnOffSlider::paintEvent(QPaintEvent *)
 {
    StyleOptionOnOff option;
    option.initFrom(this);

    option.position = m_position;
    option.marked = true;

    QPainter painter(this);
    style()->drawControl(QStyle::CE_CheckBox, &option, &painter, this);
}

