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
    QCheckBox(parent) {
}


void OnOffSlider::mouseReleaseEvent(QMouseEvent *e) {
    this->setChecked(!this->isChecked());
}

void OnOffSlider::mousePressEvent(QMouseEvent *) {
}

void OnOffSlider::paintEvent(QPaintEvent * e)
 {
    StyleOptionOnOff option;
    option.initFrom(this);

    option.position = this->isChecked() ? 1 : 0;

    QPainter painter(this);
    style()->drawControl(QStyle::CE_CheckBox, &option, &painter, this);
}

