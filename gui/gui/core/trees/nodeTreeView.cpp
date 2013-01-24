/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "nodeTreeView.h"

#include <QtGui>
#include <boost/shared_ptr.hpp>

#include <debug/cauv_debug.h>

#include "model/nodes/numericnode.h"
#include "delegates/delegate.h"
#include "model/nodeItemModel.h"

using namespace cauv;
using namespace cauv::gui;

NodeTreeView::NodeTreeView(QWidget * parent) :
    QTreeView(parent),
    m_fixedSize(false)
{
    init();
}

NodeTreeView::NodeTreeView(bool fixedSize,
                           QWidget * parent) :
    QTreeView(parent),
    m_fixedSize(fixedSize) {
    init();
}

void NodeTreeView::init() {
    header()->hide();
    setSelectionBehavior(QAbstractItemView::SelectRows);
    //setAnimated(true);
    setSelectionMode(QAbstractItemView::SingleSelection);
    setVerticalScrollMode(QAbstractItemView::ScrollPerPixel);
    setEditTriggers(QAbstractItemView::EditKeyPressed);

    m_delegateMap = boost::make_shared<DelegateProxy>(this);
    setItemDelegateForColumn(0, m_delegateMap.get());


    setRootIsDecorated(false);
    setDragEnabled(true);
    setDropIndicatorShown(true);
    setAcceptDrops(false);

    QPalette p = this->palette();
    p.setColor(QPalette::Highlight, QColor(0,0,0,0));
    p.setColor(QPalette::Background, QColor(0,0,0,0));
    p.setColor(QPalette::HighlightedText, QColor(0,0,0,255));
    this->setPalette(p);
    this->viewport()->setPalette(p);
    this->setBackgroundRole(QPalette::Background);
    this->viewport()->setBackgroundRole(QPalette::Background);
    this->setFrameShape(QFrame::NoFrame);
    this->setAutoFillBackground(false);
    this->setMinimumSize(0, 0);
    this->viewport()->setMinimumSize(0,0);
    setIndentation(3);

    this->setExpandsOnDoubleClick(false);

    if(m_fixedSize){
        this->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        this->viewport()->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    }
}

void NodeTreeView::sizeToFit(){
    setExpanded(rootIndex(), true);
    updateGeometry();
    resize(sizeHint());
}

void NodeTreeView::registerDelegate(node_type nodeType,
                                    boost::shared_ptr<AbstractNodeDelegate> delegate){
    m_delegateMap->registerDelegate(nodeType, delegate);
}

void NodeTreeView::setModel(QAbstractItemModel *model){
    QTreeView::setModel(model);
    if(m_fixedSize) {
        connect(model, SIGNAL(layoutChanged()), this, SLOT(sizeToFit()));
    }
    setExpanded(rootIndex(), true);
    sizeToFit();
}

void NodeTreeView::mouseReleaseEvent(QMouseEvent *event){
    QModelIndex index = indexAt(event->pos());
    if(!index.isValid()) {
        return;
    }
    if(state() != QAbstractItemView::DraggingState &&
       state() != QAbstractItemView::AnimatingState){
        QStyleOptionViewItem option;
        option.initFrom(this);
        option.rect = this->visualRect(index);
        if(m_delegateMap->controlRect(option, index).contains(event->pos()) &&
                (model()->flags(index) & Qt::ItemIsEditable))
            edit(index);
        else toggleExpanded(index);
    } else {
        QTreeView::mouseReleaseEvent(event);
    }
}


QSize NodeTreeView::sizeHint() const {
    QModelIndex root = rootIndex();
    int height = 0;
    int width = 0;

    for(int i = 0; i < model()->rowCount(root); i++){
        QSize rowSize = sizeHint(root.child(i, 0));
        height += rowSize.height();
        width = std::max(width, rowSize.width());
    }

    //qDebug() << "computed size hint = " << QSize(width, height);

    return QSize(width, height);
}


QSize NodeTreeView::sizeHint(QModelIndex index) const {
    if(!index.isValid()){
        return QSize(0, 0);
    }

    int rows = model()->rowCount(index);
    QStyleOptionViewItem option;
    option.initFrom(this);
    QSize size = m_delegateMap->sizeHint(option, index);
    int height = size.height();
    int width = size.width();

    if(index == rootIndex() || isExpanded(index)) {
        for(int i = 0; i < rows; i++){
            QSize rowSize = sizeHint(index.child(i, 0));
            height += rowSize.height();
            width = std::max(width, rowSize.width());
        }
    }

    return QSize(width + this->indentation(), height);
}

void NodeTreeView::toggleExpanded(QModelIndex const& index){
    //info() << "pre-inversion" << isExpanded(index);
   // qDebug() << "sizeHint = " << sizeHint();
    setExpanded(index, !isExpanded(index));
    //info() << "post-inversion" << isExpanded(index);
    //qDebug() << "sizeHint = " << sizeHint();
    //sizeToFit();
}

void NodeTreeView::keyPressEvent(QKeyEvent *event){
    Q_EMIT onKeyPressed(event);
    QTreeView::keyPressEvent(event);
}

void NodeTreeView::applyFilters(){
    debug(8) << "applyFilters()";
    applyFilters(rootIndex());
}

void NodeTreeView::applyFilters(QModelIndex const& index){
    debug(8) << "applyFilters(QModelIndex const&)";

    boost::shared_ptr<Node> node = static_cast<Node*>(index.internalPointer())->shared_from_this();

    debug(8) << "filtering" << node->nodePath();

    // apply to current node
    if (!applyFilters(node))
        this->setRowHidden(index.row(), index.parent(), true);
    else this->setRowHidden(index.row(), index.parent(), false);

    // recurse on children
    for (int i = 0; i < model()->rowCount(index); i++) {
        applyFilters(index.child(i, 0));
    }
}

bool NodeTreeView::applyFilters(boost::shared_ptr<Node> const& node){
    debug(8) << "applyFilters(boost::shared_ptr<Node> node)";
    foreach(boost::shared_ptr<NodeFilterInterface> const& filter, m_filters){
        // filtering is exclusive, so if any filter says no then the
        // node won't appear in the list
        if (!filter->filter(node)) return false;
    }
    return true;
}

void NodeTreeView::registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter){
    m_filters.push_back(filter);
    // all filters should have a filterChanged signal, but it's not actually enforced
    // by the interface as Qt doesn't handle multiple inhertiance for QObject
    if(QObject * object = dynamic_cast<QObject *>(filter.get())){
        object->connect(object, SIGNAL(filterChanged()), this, SLOT(applyFilters()));
    }
}
