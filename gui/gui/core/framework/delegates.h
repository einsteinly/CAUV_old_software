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

#include <QObject>
#include <QStyledItemDelegate>

#include <gui/core/model/node.h>

namespace cauv {
namespace gui {


class NodeTreeView;



class NodeDelegateMapper : public QStyledItemDelegate
{
    Q_OBJECT

public:
    NodeDelegateMapper(QObject *parent = 0);

    void registerDelegate(node_type nodeType, boost::shared_ptr<QAbstractItemDelegate> delegate);

    void registerDelegate(boost::shared_ptr<Node> node,
                          boost::shared_ptr<QAbstractItemDelegate> delegate);

    boost::shared_ptr<QAbstractItemDelegate> getDelegate(boost::shared_ptr<Node> node) const;

    QWidget * createEditor ( QWidget * parent, const QStyleOptionViewItem & option,
                             const QModelIndex & index ) const;

    void setEditorData(QWidget *editor, const QModelIndex &index) const;


    void updateEditorGeometry(QWidget *editor,
                              const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;

    void setSizeHint(const QSize size);

protected:
    std::map<boost::shared_ptr<Node>, boost::shared_ptr<QAbstractItemDelegate> > m_delegates;
    std::map<node_type, boost::shared_ptr<QAbstractItemDelegate> > m_default_delegates;
    QSize m_sizeHint;
    bool m_hasSizeHint;
};




class NumericDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    NumericDelegate(QObject *parent=0);

    void paint(QPainter *painter, const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    void setEditorData(QWidget *editor, const QModelIndex &index) const;

    QWidget * createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const;

protected Q_SLOTS:
    void commit();

protected:
    NodeTreeView * m_view;
    mutable std::set<QModelIndex> m_hasEditor;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_DELEGATES_H__
