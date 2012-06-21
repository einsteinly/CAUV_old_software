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

#include "radialSegment.h"
#include "radialMenuItem.h"
#include "style.h"

using namespace liquid;
using namespace liquid::magma;

RadialMenuItem::RadialMenuItem(QModelIndex const& index,
                               RadialSegment * segment,
                               Qt::WindowFlags f
        ):
    QLabel(segment, f),
    m_index(index),
    m_segment(segment) {
    init();
}

RadialMenuItem::RadialMenuItem(QModelIndex const& index,
                               RadialSegment * segment,
                               const QString& text,
                               Qt::WindowFlags f
        ):
    QLabel(text, segment, f),
    m_index(index),
    m_segment(segment) {
    init();
}

void RadialMenuItem::init(){
    this->setMouseTracking(true);
    m_hoverTimer.setInterval(100);
    m_hoverTimer.setSingleShot(true);
    m_hoverTimer.connect(&m_hoverTimer, SIGNAL(timeout()), this, SIGNAL(itemHovered()));
}

void RadialMenuItem::enterEvent(QEvent *){
    m_hoverTimer.start();
}

void RadialMenuItem::mouseReleaseEvent(QMouseEvent *){
    Q_EMIT itemSelected();
}

void RadialMenuItem::leaveEvent(QEvent *){
    m_hoverTimer.stop();
}

QModelIndex RadialMenuItem::index() const {
    return m_index;
}

RadialSegment * RadialMenuItem::segment() const {
    return m_segment;
}

void RadialMenuItem::setAngle(float angle){
    m_angle = angle;
}

float RadialMenuItem::getAngle() const{
    return m_angle;
}
