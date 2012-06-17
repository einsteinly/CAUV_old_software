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
#include "style.h"

#include <debug/cauv_debug.h>
#include <QDebug>

using namespace liquid;
using namespace liquid::magma;

RadialMenu::RadialMenu(RadialMenuStyle const& style, QWidget *parent)
    : QAbstractItemView(parent), m_style(style)
{
    this->setFocusPolicy(Qt::StrongFocus);

    setAttribute(Qt::WA_TranslucentBackground);
    viewport()->setAttribute(Qt::WA_TranslucentBackground);
    // WA_MacNoShadow introduced in Qt 4.8
    #if QT_VERSION > 0x408000
    setAttribute(Qt::WA_MacNoShadow);
    viewport()->setAttribute(Qt::WA_MacNoShadow);
    #else
    #warning Radial menus may not work
    #endif 
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setFrameStyle(QFrame::NoFrame);
}

void RadialMenu::focusOutEvent(QFocusEvent *){
    deleteLater();
}

void RadialMenu::recomputeMask(){
    // union all the masks of our children
    QRegion maskedRegion;
    foreach(QObject * child, viewport()->children()){
        if(QWidget * widget = qobject_cast<QWidget*>(child)){
            // childrens masks will be in children coords
            // so map them to view coords
            QPoint offset = widget->mapToParent(QPoint(0,0));
            QRegion region = widget->mask().translated(offset);
            //qDebug() << "child region" << region;
            maskedRegion = maskedRegion.united(region);
        }
    }
    setMask(maskedRegion);
    viewport()->setMask(maskedRegion);
    //qDebug() << "new mask " << maskedRegion;
}


void RadialMenu::resizeEvent(QResizeEvent * /* event */)
{
    recomputeMask();
}

void RadialMenu::fitToContents() {
    QRect boundingRect = viewport()->childrenRect();

    // move the children within the widget so that if we expand or
    // shrink it they don't actually move on screen
    QPoint translation = boundingRect.topLeft();
    foreach(QObject * child, viewport()->children()){
        if(QWidget * widget = qobject_cast<QWidget*>(child))
            widget->setGeometry(widget->geometry().translated(-translation));
    }

    // move the widget so it covers the whole area used by its children
    // going from childrencoords -> viewport coords -> parent coords
    QPoint tl = mapToParent(viewport()->mapToParent(boundingRect.topLeft()));
    this->move(tl);
    // and resize. We'll update the view and a viewport just to make sure
    viewport()->setFixedSize(viewport()->childrenRect().size());
    setFixedSize(viewport()->childrenRect().size());

    // masks might have moved
    recomputeMask();
}


/* QAbstractItemModel implementation */


QRect RadialMenu::visualRect ( const QModelIndex & index ) const{
    Q_UNUSED(index);
    debug() << "visualRect()";
    return rect();
}

void RadialMenu::setModel( QAbstractItemModel * model ) {
    QAbstractItemView::setModel(model);
    m_segmentMap.clear();
    foreach(QObject * child, this->children()){
        if(qobject_cast<RadialSegment*>(child))
            delete child;
    }

    // force creation of root element


    scrollTo(model->index(0, 0, QModelIndex()).child(0,0));
    //scrollTo(QModelIndex());

}

int RadialMenu::depthOfIndex(const QModelIndex& index){
    QModelIndex i = index;
    int count = 0;
    while((i = i.parent()) != QModelIndex()) {
        count++;
    }
    return count;
}

void RadialMenu::scrollTo(const QModelIndex& itemIndex, QAbstractItemView::ScrollHint){
    debug() << "scrollTo()";

    QModelIndex index = itemIndex;
    int depth = depthOfIndex(itemIndex);

    debug() << "index depth" <<depth;

    do {
        // make sure we've created a segment for this item to live in
        if (!m_segmentMap.contains(index.parent())) {
            m_segmentMap[index.parent()] = new RadialSegment(Default_RadialSegmentStyle(),
                m_style.centreSpace + (depth + 1) * ((m_style.spacing/2) + (m_style.segment.width/2)),
                (depth+1)*40, (depth+1)*80, viewport());
            connect(m_segmentMap[index.parent()], SIGNAL(maskComputed(QRegion)), this, SLOT(recomputeMask()));
        }

        RadialSegment * c = m_segmentMap[index.parent()];
        c->move(-(c->width()/2), -c->height()/2);
        c->show();
        //qDebug() << depth << "segment geometry set to " << c->geometry();

        index = index.parent();
        depth--;
    } while (index != QModelIndex());

    fitToContents();
}

QModelIndex RadialMenu::indexAt(const QPoint& p) const{
    debug() << "indexAt()";
    QWidget * pick = viewport()->childAt(p);
    if (RadialSegment * segment = qobject_cast<RadialSegment*>(pick)){
        return segment->indexAt(p);
    }

    return QModelIndex();
}

QModelIndex RadialMenu::moveCursor(QAbstractItemView::CursorAction, Qt::KeyboardModifiers){
    debug() << "moveCursor()";
    return QModelIndex();
}

int RadialMenu::horizontalOffset() const{
    debug() << "horizontalOffset()";
    return 0;
}

int RadialMenu::verticalOffset() const{
    debug() << "verticalOffset()";
    return 0;
}

bool RadialMenu::isIndexHidden(const QModelIndex&) const{
    debug() << "isIndexHidden()";
    return false;
}

void RadialMenu::setSelection(const QRect&, QFlags<QItemSelectionModel::SelectionFlag>){
    debug() << "setSelection()";

}

QRegion RadialMenu::visualRegionForSelection(const QItemSelection&) const{
    debug() << "visualRegionForSelection()";
    return QRegion(rect());
}

