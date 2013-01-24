/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */



#include <QtGui>

#include "radialMenu.h"
#include "radialSegment.h"
#include "radialMenuItem.h"
#include "style.h"

#include <debug/cauv_debug.h>

using namespace liquid;
using namespace liquid::magma;

RadialMenu::RadialMenu(RadialMenuStyle const& style,
                       int titleRole,
                       QWidget *parent) :
    QAbstractItemView(parent),
    m_style(style),
    m_role(titleRole)
{
    this->setFocusPolicy(Qt::StrongFocus);

    setAttribute(Qt::WA_TranslucentBackground);
    viewport()->setAttribute(Qt::WA_TranslucentBackground);

    // WA_MacNoShadow introduced in Qt 4.8
#if QT_VERSION < 0x040800
#warning Radial menus may not work. update to Qt >= 4.8
#else
    setAttribute(Qt::WA_MacNoShadow);
    viewport()->setAttribute(Qt::WA_MacNoShadow);
#endif
    setWindowFlags(windowFlags() | Qt::FramelessWindowHint | Qt::WindowStaysOnTopHint);
    setFrameStyle(QFrame::NoFrame);
    setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    resize(500, 500); // will grow as needed
}


QSize RadialMenu::sizeHint() const{
    return size();
}

void RadialMenu::focusOutEvent(QFocusEvent *){
    deleteLater();
}

void RadialMenu::fitToContents() {

    //info() << "resizing to contents";

    // move the children within the widget so that if we expand
    // it they don't actually move on screen
    foreach(QObject * child, viewport()->children()){
        if(QWidget * widget = qobject_cast<QWidget*>(child)) {
            widget->move(rect().center() - widget->rect().center());
        }
    }


    QRect boundingRect = viewport()->childrenRect().united(rect());
    //qDebug() << "boudingRect" << boundingRect;
    // move the widget so it covers the whole area used by its children
    // going from childrencoords -> viewport coords -> parent coords
    QPoint tl = mapToParent(viewport()->mapToParent(boundingRect.topLeft()));
    this->move(tl);
    // and resize
    resize(boundingRect.size());


    // move the children within the widget so that if we expand
    // it they don't actually move on screen
    foreach(QObject * child, viewport()->children()){
        if(QWidget * widget = qobject_cast<QWidget*>(child)) {
            widget->move(rect().center() - widget->rect().center());
        }
    }


}


/* QAbstractItemModel implementation */


QRect RadialMenu::visualRect ( const QModelIndex & index ) const{
    Q_UNUSED(index);
    //debug() << "visualRect()";
    return rect();
}

void RadialMenu::setModel( QAbstractItemModel * model ) {
    QAbstractItemView::setModel(model);
    m_segmentMap.clear();
    foreach(QObject * child, this->children()){
        if(qobject_cast<RadialSegment*>(child))
            delete child;
    }
}

void RadialMenu::showEvent(QShowEvent *e){
    QAbstractItemView::showEvent(e);
    // force creation of root element
    //scrollTo(model()->index(0, 0, QModelIndex()).child(0,0));
    //scrollTo(QModelIndex());
    scrollTo(rootIndex());
}

int RadialMenu::depthOfIndex(const QModelIndex& index) const{
    QModelIndex i = index;
    int count = 1;
    while(i != rootIndex()) {
        if(!i.isValid()) break;
        count++;
        i = i.parent();
    }
    return count;
}


RadialSegment * RadialMenu::newRadialSegmentFor(QModelIndex index,
                                                RadialMenuItem * parent) const{

    RadialSegment * segment = new RadialSegment(
                Default_RadialSegmentStyle(),
                parent?0:1,
                viewport());

    // stack under parent so parent events don't get
    // absorbed by higher up widgets
    if(parent)
        segment->stackUnder(parent->segment());

    float radius = m_style.centreSpace + m_style.segment.width;
    if(parent) {
        radius = parent->segment()->getRadius() +
                m_style.spacing +
                m_style.segment.width;
    }
    segment->setRadius(radius);

    // centre it in the RadialMenu widget
    //segment->move(this->rect().center());
    //segment->move(segment->x()-ceil(radius), segment->y()-ceil(radius));

    // connect stuff up
    connect(segment, SIGNAL(itemHovered(RadialMenuItem*)),
            this, SLOT(scrollTo(RadialMenuItem*)));
    connect(segment, SIGNAL(itemSelected(RadialMenuItem*)),
            this, SLOT(itemSelected(RadialMenuItem*)));
    connect(segment, SIGNAL(sizeChanged(QSize)),
            this, SLOT(fitToContents()));

    // add items for each of its valid children
    foreach(QModelIndex const& i, validChildren(index)){
        segment->addItem(new RadialMenuItem(
                         i,
                         segment,
                         model()->data(i, m_role).toString()));
    }

    if (parent) {
        info() << "set parent angle!" << parent->getRotation();
        segment->setRotation(parent->getRotation() - (segment->getAngle()/2) -90);
    }
    else {
        info() << "parent not set, can't set anlge";
        segment->setRotation(((360 - m_style.segment.maxAngle)/2) + 90);
    }


    return segment;
}

void RadialMenu::itemSelected(RadialMenuItem * item){
    Q_EMIT indexSelected(item->index());
    deleteLater();
}

void RadialMenu::scrollTo(RadialMenuItem * item){
    //hideEverythingAbove(item->index());
    recursiveOpen(item->index(), item);
    fitToContents();
}

void RadialMenu::scrollTo(const QModelIndex& index, QAbstractItemView::ScrollHint){
    //hideEverythingAbove(index);
    recursiveOpen(index); //todo: map from index to RadialMenuItem to pass in parent
    fitToContents();
}

void RadialMenu::hideEverythingAbove(QModelIndex const& index) {
    int depth = depthOfIndex(index);

    // hide levels above depth
     while (m_stack.count()-1 > depth) {
        //qDebug() << "hiding " << &m_stack.top();
        m_stack.top()->hide();
        m_stack.pop();
    }
}

void RadialMenu::recursiveOpen(const QModelIndex& index, RadialMenuItem *parent){
    //debug() << "scrollTo()";

    if (!index.isValid() || model()->rowCount(index) == 0)
        return;

    if(index != rootIndex())
        recursiveOpen(index.parent(), parent);

    // make sure we've created a segment for this item to live in
    if (!m_segmentMap.contains(index)) {
        info() << "creating new radial segment";
        m_segmentMap[index] = newRadialSegmentFor(index, parent);
    }

    m_stack.push(m_segmentMap[index]);

    if(!m_segmentMap[index]->isVisible())
        m_segmentMap[index]->animateIn();
    else info() << "item already visisble";
}

QModelIndex RadialMenu::indexAt(const QPoint& p) const{
    //debug() << "indexAt()";
    QWidget * pick = viewport()->childAt(p);
    if(!pick) return QModelIndex();

    if (RadialSegment * segment = qobject_cast<RadialSegment*>(pick)){
        const RadialMenuItem * item = segment->itemAt(p);
        if(item)
            return item->index();
    }
    return QModelIndex();
}

QModelIndex RadialMenu::moveCursor(QAbstractItemView::CursorAction, Qt::KeyboardModifiers){
    //debug() << "moveCursor()";
    return QModelIndex();
}

int RadialMenu::horizontalOffset() const{
    //debug() << "horizontalOffset()";
    return 0;
}

int RadialMenu::verticalOffset() const{
    //debug() << "verticalOffset()";
    return 0;
}

bool RadialMenu::isIndexHidden(const QModelIndex&) const{
    //debug() << "isIndexHidden()";
    return false;
}

void RadialMenu::setSelection(const QRect&, QFlags<QItemSelectionModel::SelectionFlag>){
    //debug() << "setSelection()";

}

QRegion RadialMenu::visualRegionForSelection(const QItemSelection&) const{
    //debug() << "visualRegionForSelection()";
    return QRegion(rect());
}

const QList<QModelIndex> RadialMenu::validChildren(QModelIndex const& index) const {
    QList<QModelIndex> selection;
    int children = model()->rowCount(index);
    for(int i = 0; i < children; i++) {
        // Assumption: all items shown in the menu are in column 0
        QModelIndex child = index.child(i, 0);
        if(child.isValid())
            selection << child;
    }
    return selection;
}

