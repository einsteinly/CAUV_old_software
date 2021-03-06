/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_DELEGATE_H__
#define __CAUV_GUI_DELEGATE_H__

#include <QtGui>

#include <model/nodeType.h>

#include <boost/shared_ptr.hpp>

namespace cauv {
namespace gui {

class Node;
class NodeTreeView;
class AbstractNodeDelegate;
class SizedDelegate;

class DelegateProxy : public QStyledItemDelegate
{
    Q_OBJECT

public:
    DelegateProxy(QObject *parent = 0);

    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<AbstractNodeDelegate> delegate);

    void registerDelegate(boost::shared_ptr<Node> node,
                          boost::shared_ptr<AbstractNodeDelegate> delegate);

    boost::shared_ptr<AbstractNodeDelegate> getDelegate(boost::shared_ptr<Node> node) const;

    QRect controlRect(const QStyleOptionViewItem &option,
                      const QModelIndex &index) const;

    QWidget * createEditor ( QWidget * parent,
                             const QStyleOptionViewItem & option,
                             const QModelIndex & index ) const;

    void setEditorData(QWidget *editor,
                       const QModelIndex &index) const;

    void updateEditorGeometry(QWidget *editor,
                              const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

    void paint(QPainter *painter,
               const QStyleOptionViewItem &option,
               const QModelIndex &index) const;

    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;

protected:
    std::map<boost::shared_ptr<Node>, boost::shared_ptr<AbstractNodeDelegate> > m_delegates;
    std::map<node_type, boost::shared_ptr<AbstractNodeDelegate> > m_default_delegates;
    boost::shared_ptr<SizedDelegate> m_default;
};


class AbstractNodeDelegate : public QStyledItemDelegate {
public:
    AbstractNodeDelegate(QObject *parent=0);

    virtual QSize sizeHint(const QStyleOptionViewItem &option,
                           const QModelIndex &index) const = 0;

    virtual QRect controlRect(const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

    virtual const QRect titleRect(const QStyleOptionViewItem& option,
                                  const QModelIndex & index ) const;

    void updateEditorGeometry(QWidget *editor,
                              const QStyleOptionViewItem &option,
                              const QModelIndex &index) const;

    void setEditorData(QWidget *editor,
                       const QModelIndex &index) const;

    virtual void paint(QPainter *painter,
                       const QStyleOptionViewItem &option,
                       const QModelIndex &index) const;
protected:
    QFont m_font;
    int m_minimumTitleWidth;
    QPen m_titlePen;
    bool m_updatesWhileEditing;
};

class SizedDelegate : public AbstractNodeDelegate {
public:
    SizedDelegate(QSize sizeHint, QObject *parent=0);
    QSize sizeHint(const QStyleOptionViewItem &option,
                   const QModelIndex &index) const;
    virtual void paint(QPainter *painter,
                       const QStyleOptionViewItem &option,
                       const QModelIndex &index) const;
protected:
protected:
    QSize m_sizeHint;
};


} // namespace gui
} // namespace cauv


#endif // __CAUV_DELEGATES_H__
