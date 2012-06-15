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

#include <QtGui>

#include "radialMenu.h"
#include "radialSegment.h"

using namespace liquid;
using namespace liquid::magma;

RadialMenu::RadialMenu(QWidget *parent)
    : QWidget(parent, Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint)
{
    setContextMenuPolicy(Qt::CustomContextMenu);
    this->connect(this, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(createSubmenu(QPoint)));
    this->setFocusPolicy(Qt::StrongFocus);
}

void RadialMenu::focusOutEvent(QFocusEvent *){
    deleteLater();
}

void RadialMenu::createSubmenu(QPoint p){
    ShapedClock * c = new ShapedClock(this);
    c->show();
    recomputeMask();
}

void RadialMenu::recomputeMask(){
    QRegion maskedRegion;
    foreach(QObject * child, this->children()){
        QWidget * widget = qobject_cast<QWidget*>(child);
        if(widget)
            maskedRegion.unite(widget->mask());
    }
    setMask(maskedRegion);
}

void RadialMenu::resizeEvent(QResizeEvent * /* event */)
{
    recomputeMask();
}

QSize RadialMenu::sizeHint() const
{
    return QSize(100, 100);
}

