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

#ifndef __CAUV_DELEGATES_H__
#define __CAUV_DELEGATES_H__

#include <QtGui>

#include <gui/core/model/nodeType.h>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class Node;
class NodeTreeView;


class NodeDelegate : public QStyledItemDelegate {
public:
    NodeDelegate(QObject *parent=0);

    virtual bool providesTitle(const QStyleOptionViewItem &option,
                               const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;
};

class DefaultNodeDelegate : public QStyledItemDelegate
{
    Q_OBJECT

public:
    DefaultNodeDelegate(QObject *parent = 0);

    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<NodeDelegate> delegate);

    void registerDelegate(boost::shared_ptr<Node> node,
                          boost::shared_ptr<NodeDelegate> delegate);

    boost::shared_ptr<NodeDelegate> getDelegate(boost::shared_ptr<Node> node) const;

    QWidget * createEditor ( QWidget * parent,
                             const QStyleOptionViewItem & option,
                             const QModelIndex & index ) const;

    void setEditorData(QWidget *editor,
                       const QModelIndex &index) const;

    const QRect titleRect(const QStyleOptionViewItem& option,
                          const QModelIndex & index ) const;

    const QRect childRect(const QStyleOptionViewItem& option,
                          const QModelIndex & index ) const;

    void updateEditorGeometry(QWidget *editor,
                              const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

    int split(const QStyleOptionViewItem &option,
              const QModelIndex &index) const;

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;

protected:
    std::map<boost::shared_ptr<Node>, boost::shared_ptr<NodeDelegate> > m_delegates;
    std::map<node_type, boost::shared_ptr<NodeDelegate> > m_default_delegates;
    QFont m_font;
};

class ShortDelegate : public NodeDelegate {
public:
    ShortDelegate(QObject *parent=0);
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;
};

class TallDelegate : public NodeDelegate {
public:
    TallDelegate(QObject *parent=0);
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;
};


class NumericDelegate : public ShortDelegate {
    Q_OBJECT
public:
    NumericDelegate(bool showTitle = true, QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    void setEditorData(QWidget *, const QModelIndex &) const;

    virtual bool providesTitle(const QStyleOptionViewItem &option,
                               const QModelIndex &index) const;
protected Q_SLOTS:
    void commit();

protected:
    bool m_titles;
};


class BooleanDelegate : public ShortDelegate {
    Q_OBJECT
public:
    BooleanDelegate(QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    QWidget * createEditor(QWidget *parent,
                           const QStyleOptionViewItem &option,
                           const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;

protected Q_SLOTS:
    void commit();
};


class ColourDelegate : public ShortDelegate {
    Q_OBJECT
public:
    ColourDelegate(QObject *parent=0);

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    void setEditorData(QWidget *editor,
                       const QModelIndex &index) const;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_DELEGATES_H__
