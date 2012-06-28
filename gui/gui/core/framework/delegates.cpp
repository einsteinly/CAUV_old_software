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

#include "delegates.h"

#include <QtGui>

#include <algorithm>

#include "model/nodes/numericnode.h"
#include "model/nodes/stringnode.h"
#include "model/nodes/groupingnode.h"

#include "widgets/neutralspinbox.h"
#include "widgets/onoff.h"
#include "framework/style.h"

using namespace cauv;
using namespace cauv::gui;


DefaultNodeDelegate::DefaultNodeDelegate(QObject *):
    m_font("Verdana", 12, 1){

    registerDelegate(nodeType<BooleanNode>(), boost::make_shared<BooleanDelegate>());
    registerDelegate(nodeType<NumericNodeBase>(), boost::make_shared<NumericDelegate>());
    registerDelegate(nodeType<StringNode>(), boost::make_shared<TallDelegate>());
    registerDelegate(nodeType<GroupingNode>(), boost::make_shared<ShortDelegate>());
}

void DefaultNodeDelegate::registerDelegate(node_type nodeType,
                                           boost::shared_ptr<NodeDelegate> delegate){
    m_default_delegates[nodeType] = delegate;
    connect(delegate.get(), SIGNAL(commitData(QWidget*)), this, SIGNAL(commitData(QWidget*)));
    connect(delegate.get(), SIGNAL(closeEditor(QWidget*)), this, SIGNAL(closeEditor(QWidget*)));
    connect(delegate.get(), SIGNAL(sizeHintChanged(QModelIndex)), this, SIGNAL(sizeHintChanged(QModelIndex)));
}

void DefaultNodeDelegate::registerDelegate(boost::shared_ptr<Node> node,
                                           boost::shared_ptr<NodeDelegate> delegate){
    m_delegates[node] = delegate;
}

boost::shared_ptr<NodeDelegate> DefaultNodeDelegate::getDelegate(const boost::shared_ptr<Node> node) const {
    if (m_delegates.find(node) != m_delegates.end())
        return m_delegates.at(node);
    else if (m_default_delegates.find(node->type) != m_default_delegates.end())
        return m_default_delegates.at(node->type);
    else return boost::make_shared<NodeDelegate>();
}

QWidget * DefaultNodeDelegate::createEditor ( QWidget * parent,
                                              const QStyleOptionViewItem & option,
                                              const QModelIndex & index ) const {
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
    return delegate->createEditor(parent, option, index);
}

void DefaultNodeDelegate::setEditorData(QWidget *editor,
                                        const QModelIndex &index) const{
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
    return delegate->setEditorData(editor, index);
}

void DefaultNodeDelegate::updateEditorGeometry(QWidget *editor,
                                               const QStyleOptionViewItem &option,
                                               const QModelIndex &index) const{
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    QStyleOptionViewItem newOption = option;
    newOption.rect = childRect(option, index);
    boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
    if(delegate->providesTitle(option, index))
        return delegate->updateEditorGeometry(editor, option, index);
    return delegate->updateEditorGeometry(editor, newOption, index);
}

const QRect DefaultNodeDelegate::titleRect(const QStyleOptionViewItem& option,
                                           const QModelIndex &index) const{
    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();

    // add some padding
    QRect out = option.rect;
    out.setLeft(out.left()+6);
    out.setRight(out.right() -6);
    out.setTop(out.top()+2);

    // and enough space for the text
    QFontMetrics fm(m_font);
    int titleHeight = fm.height();
    int titleWidth = fm.width(QString::fromStdString(node->nodeName()));

    return QRect(out.x(), out.y(), titleWidth, titleHeight+2);
}

const QRect DefaultNodeDelegate::childRect(const QStyleOptionViewItem& option,
                                           const QModelIndex &index) const{

    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();

    if(getDelegate(node)->providesTitle(option, index))
        return option.rect;

    QRect out = option.rect;
    QRect title = titleRect(option, index);

    out.setLeft(title.left());
    if (int splitPosition = split(option, index))
        out.setLeft(title.left() + splitPosition);
    else out.setTop(title.bottom());

    return out;
}

void DefaultNodeDelegate::paint(QPainter *painter,
                                const QStyleOptionViewItem &option,
                                const QModelIndex &index) const{
    // display the node
    void* ptr = index.internalPointer();
    if(!ptr){
        error() << "NodeDelegateMapper:: nothing to paint!";
        return;
    }
    const boost::shared_ptr<Node> node = static_cast<Node*>(ptr)->shared_from_this();
    if (node) {
        QStyleOptionViewItem newOption = option;
        newOption.rect = childRect(option, index);

        painter->setFont(m_font);
        QRectF title = titleRect(option, index);
        //painter->fillRect(title, Qt::green);

        QString name = QString::fromStdString(node->nodeName());
        painter->setPen(Qt::black);
        QTextOption to(Qt::AlignVCenter);
        to.setWrapMode(QTextOption::NoWrap);
        painter->drawText(title, name, to);

        try {
            boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
            if(delegate->providesTitle(option, index))
                delegate->paint(painter, option, index);
            else delegate->paint(painter, newOption, index);
        } catch (std::out_of_range){
            QStyledItemDelegate::paint(painter, newOption, index);
        }
    } else QStyledItemDelegate::paint(painter, option, index);
}

int DefaultNodeDelegate::split(const QStyleOptionViewItem &option,
                               const QModelIndex &index) const{

    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
    if (dynamic_cast<ShortDelegate*>(delegate.get()))
        return std::max(titleRect(option, index).width(), 62);
    else return 0;
}

QSize DefaultNodeDelegate::sizeHint(const QStyleOptionViewItem &option,
                                    const QModelIndex &index) const{

    const boost::shared_ptr<Node> node = static_cast<Node*>(
                index.internalPointer())->shared_from_this();
    try {
        boost::shared_ptr<NodeDelegate> delegate = getDelegate(node);
        QSize s = delegate->sizeHint(option, index);
        s.setWidth(std::max(titleRect(option, index).width(), s.width()));
        return s;
    } catch (std::out_of_range){
        return QSize(0,25);
    }
}



NodeDelegate::NodeDelegate(QObject * parent) : QStyledItemDelegate(parent) {}

QSize NodeDelegate::sizeHint(const QStyleOptionViewItem &,
                             const QModelIndex &) const{
    return QSize(0, 25);
}


bool NodeDelegate::providesTitle(const QStyleOptionViewItem &,
                                 const QModelIndex &) const {
    return false;
}

ShortDelegate::ShortDelegate(QObject * parent) : NodeDelegate(parent) {}

QSize ShortDelegate::sizeHint(const QStyleOptionViewItem &,
                              const QModelIndex &) const{
    return QSize(200, 25);
}

TallDelegate::TallDelegate(QObject * parent) : NodeDelegate(parent) {}

QSize TallDelegate::sizeHint(const QStyleOptionViewItem &,
                             const QModelIndex &) const{
    return QSize(200, 38);
}


NumericDelegate::NumericDelegate(bool showTitle, QObject *parent) :
    ShortDelegate(parent),
    m_titles(showTitle) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Int,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::UInt,
                            new QItemEditorCreator<NeutralSpinBox>("value"));
    factory->registerEditor(QVariant::Double,
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<float>(),
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
    factory->registerEditor((QVariant::Type)qMetaTypeId<BoundedFloat>(),
                            new QItemEditorCreator<NeutralDoubleSpinBox>("value"));
}

bool NumericDelegate::providesTitle(const QStyleOptionViewItem &,
                                    const QModelIndex &) const{
    return true;
}

void NumericDelegate::paint(QPainter *painter,
                            const QStyleOptionViewItem &option,
                            const QModelIndex &index) const
{
    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());
    if (node) {
        QString value = QString::number( node->asNumber().toDouble(),
                                         'g', node->getPrecision() );
        value.append(QString::fromStdString(node->getUnits()));

        QString text;

        if(m_titles)
            text = QString::fromStdString(node->nodeName()).append(":").append(value);
        else text = value;

        QStyleOptionProgressBarV2 progressBarOption;
        progressBarOption.rect = option.rect;
        progressBarOption.text = text;
        progressBarOption.textVisible = true;
        progressBarOption.invertedAppearance = node->isInverted();
        progressBarOption.palette.setColor(QPalette::Text, Qt::black);
        if(!node->isMutable())
            progressBarOption.palette.setColor(QPalette::Text, QColor(100,100,100));

        progressBarOption.maximum = (node->isMaxSet() && node->isMinSet()) ? 1000 : 0;
        progressBarOption.minimum = 0;
        progressBarOption.progress = (int)(fabs(pivot(node->getMin().toDouble(),
                                                      node->getNeutral().toDouble(),
                                                      node->getMax().toDouble(),
                                                      node->asNumber().toDouble()))*progressBarOption.maximum);

        QApplication::style()->drawControl(QStyle::CE_ProgressBar,
                                           &progressBarOption, painter);
    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }
}

void NumericDelegate::commit() {
    emit commitData(static_cast<QWidget *>(sender()));
}

void NumericDelegate::setEditorData(QWidget *editor,
                                    const QModelIndex &index) const{
    // don't allow updates while editing
    if(editor->property("data-initialised").toBool()) return;
    editor->setProperty("data-initialised", true);

    QStyledItemDelegate::setEditorData(editor, index);

    NumericNodeBase * node = dynamic_cast<NumericNodeBase*>((Node*)index.internalPointer());

    if(node && node->isMaxSet() && node->isMinSet()) {
        if(NeutralSpinBox * neutral = qobject_cast<NeutralSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toInt());
            neutral->setMaximum(node->getMax().toInt());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toInt());
            neutral->setInverted(node->isInverted());
        }

        if(NeutralDoubleSpinBox * neutral = qobject_cast<NeutralDoubleSpinBox*>(editor)){
            neutral->setMinimum(node->getMin().toDouble());
            neutral->setMaximum(node->getMax().toDouble());
            neutral->setWrapping(node->getWraps());
            neutral->setNeutral(node->getNeutral().toDouble());
            neutral->setDecimals(node->getPrecision());
            neutral->setInverted(node->isInverted());
        }
    }
}



BooleanDelegate::BooleanDelegate(QObject * parent) :
    ShortDelegate(parent) {
    QItemEditorFactory * factory = new QItemEditorFactory();
    setItemEditorFactory(factory);
    factory->registerEditor(QVariant::Bool, new QItemEditorCreator<OnOffSlider>("checked"));
}


void BooleanDelegate::paint(QPainter *painter,
                            const QStyleOptionViewItem &option,
                            const QModelIndex &index) const
{
    BooleanNode * node = dynamic_cast<BooleanNode*>((Node*)index.internalPointer());
    if (node) {
        StyleOptionOnOff onOffOption;
        onOffOption.rect = option.rect;
        onOffOption.position = node->get();
        onOffOption.marked = node->isMutable();
        QApplication::style()->drawControl(QStyle::CE_CheckBox,
                                           &onOffOption, painter);
    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }
}


QWidget * BooleanDelegate::createEditor(QWidget *parent,
                                        const QStyleOptionViewItem &option,
                                        const QModelIndex &index) const{
    QWidget *retval = QStyledItemDelegate::createEditor(parent, option, index);
    if (OnOffSlider * slider = dynamic_cast<OnOffSlider*>(retval)) {
        slider->setAnimation(true);
        connect(retval, SIGNAL(switched()), this, SLOT(commit()));
        BooleanNode * node = dynamic_cast<BooleanNode*>((Node*)index.internalPointer());
        node->set(!node->get());
    }
    return retval;
}

void BooleanDelegate::commit() {
    emit commitData(static_cast<QWidget *>(sender()));
}
