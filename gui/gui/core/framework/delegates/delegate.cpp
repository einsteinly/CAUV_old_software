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

#include <algorithm>

#include "model/nodes/numericnode.h"
#include "model/nodes/stringnode.h"
#include "model/nodes/groupingnode.h"
#include "model/nodes/colournode.h"

#include "delegate.h"
#include "numericdelegate.h"
#include "colourdelegate.h"
#include "booleandelegate.h"

using namespace cauv;
using namespace cauv::gui;


DelegateProxy::DelegateProxy(QObject *){

    registerDelegate(nodeType<BooleanNode>(), boost::make_shared<BooleanDelegate>());
    registerDelegate(nodeType<NumericNodeBase>(), boost::make_shared<NumericDelegate>());
    registerDelegate(nodeType<StringNode>(), boost::make_shared<SizedDelegate>(QSize(100, 25)));
    registerDelegate(nodeType<GroupingNode>(), boost::make_shared<SizedDelegate>(QSize(100, 25)));
    registerDelegate(nodeType<ColourNode>(), boost::make_shared<ColourDelegate>());
    m_default = boost::make_shared<SizedDelegate>(QSize(100,25));
}

void DelegateProxy::registerDelegate(node_type nodeType,
                                     boost::shared_ptr<AbstractNodeDelegate> delegate){
    m_default_delegates[nodeType] = delegate;
    connect(delegate.get(), SIGNAL(commitData(QWidget*)), this, SIGNAL(commitData(QWidget*)));
    connect(delegate.get(), SIGNAL(closeEditor(QWidget*)), this, SIGNAL(closeEditor(QWidget*)));
    connect(delegate.get(), SIGNAL(sizeHintChanged(QModelIndex)), this, SIGNAL(sizeHintChanged(QModelIndex)));
}

void DelegateProxy::registerDelegate(boost::shared_ptr<Node> node,
                                     boost::shared_ptr<AbstractNodeDelegate> delegate){
    m_delegates[node] = delegate;
}

boost::shared_ptr<AbstractNodeDelegate> DelegateProxy::getDelegate(const boost::shared_ptr<Node> node) const {
    if (m_delegates.find(node) != m_delegates.end())
        return m_delegates.at(node);
    else if (m_default_delegates.find(node->type) != m_default_delegates.end())
        return m_default_delegates.at(node->type);
    else return m_default;
}

QWidget * DelegateProxy::createEditor ( QWidget * parent,
                                        const QStyleOptionViewItem & option,
                                        const QModelIndex & index ) const {
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    return getDelegate(node)->createEditor(parent, option, index);
}

void DelegateProxy::setEditorData( QWidget *editor,
                                   const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    return getDelegate(node)->setEditorData(editor, index);
}

void DelegateProxy::updateEditorGeometry(QWidget *editor,
                                         const QStyleOptionViewItem &option,
                                         const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    return getDelegate(node)->updateEditorGeometry(editor, option, index);
}

QRect DelegateProxy::controlRect(const QStyleOptionViewItem &option,
                                 const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    return getDelegate(node)->controlRect(option, index);

}

void DelegateProxy::paint(QPainter *painter,
                          const QStyleOptionViewItem &option,
                          const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    getDelegate(node)->paint(painter, option, index);
}


QSize DelegateProxy::sizeHint(const QStyleOptionViewItem &option,
                              const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    return getDelegate(node)->sizeHint(option, index);
}



AbstractNodeDelegate::AbstractNodeDelegate(QObject * parent) :
    QStyledItemDelegate(parent),
    m_font("Verdana", 12, 1),
    m_minimumTitleWidth(62),
    m_titlePen(Qt::black),
    m_updatesWhileEditing(false) {
    m_font.setPixelSize(12);
}

const QRect AbstractNodeDelegate::titleRect(const QStyleOptionViewItem& option,
                                            const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();

    // add some padding
    QRect out = option.rect;
    out.setLeft(out.left()+6);
    out.setRight(out.right() -6);
    out.setTop(out.top());

    // and enough space for the text
    QFontMetrics fm(m_font);
    int titleHeight = fm.height();
    int titleWidth = fm.width(QString::fromStdString(node->nodeName()), -1, Qt::TextSingleLine);

    return QRect(out.x(), out.y(),
                 std::max(titleWidth, m_minimumTitleWidth),
                 std::max(titleHeight, out.height()));
}

void AbstractNodeDelegate::paint(QPainter *painter,
                                 const QStyleOptionViewItem &option,
                                 const QModelIndex &index) const{
    assert(index.internalPointer());
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();

    painter->setFont(m_font);
    QRectF title = titleRect(option, index);
    //painter->fillRect(title, Qt::green);

    QString name = QString::fromStdString(node->nodeName());
    painter->setPen(m_titlePen);
    QTextOption to(Qt::AlignVCenter);
    to.setWrapMode(QTextOption::NoWrap);
    painter->drawText(title, name, to);
}

void AbstractNodeDelegate::setEditorData(QWidget *editor,
                                         const QModelIndex &index) const {
    // don't allow updates while editing

    info() << "setEditorData";

    QStyledItemDelegate::setEditorData(editor, index);

    if(!m_updatesWhileEditing) {
        if(editor->property("data-initialised").toBool()) return;
        editor->setProperty("data-initialised", true);
    }
}

void AbstractNodeDelegate::updateEditorGeometry(QWidget *editor,
                                                const QStyleOptionViewItem &option,
                                                const QModelIndex &index) const {
    QStyleOptionViewItem modified = option;
    modified.rect = controlRect(option, index);
    QStyledItemDelegate::updateEditorGeometry(editor, modified, index);
}

QRect AbstractNodeDelegate::controlRect(const QStyleOptionViewItem &option,
                                        const QModelIndex &index) const {
    QRect rect = option.rect;
    rect.setLeft(titleRect(option, index).right());
    return rect;
}

SizedDelegate::SizedDelegate(QSize sizeHint,
                             QObject * parent) :
    AbstractNodeDelegate(parent),
    m_sizeHint(sizeHint) {
}

void SizedDelegate::paint(QPainter *painter,
                          const QStyleOptionViewItem &option,
                          const QModelIndex &index) const{
    painter->save();
    AbstractNodeDelegate::paint(painter, option, index);
    painter->restore();

    QStyleOptionViewItem modified(option);
    modified.rect = controlRect(option, index);

    painter->save();
    QStyledItemDelegate::paint(painter, modified, index);
    painter->restore();
}

QSize SizedDelegate::sizeHint(const QStyleOptionViewItem &,
                              const QModelIndex &) const{
    return QSize(150, 25);
}


