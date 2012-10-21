/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __CAUV_NODETREEVIEW_H__
#define __CAUV_NODETREEVIEW_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <filter/nodeFilterInterface.h>
#include <model/nodeType.h>

namespace cauv {
namespace gui {

class NodeItemModel;
class DelegateProxy;
class AbstractNodeDelegate;
class Node;

/**
* Filterable tree view onto the node model
*/
class NodeTreeView : public QTreeView {
    Q_OBJECT
public:
    NodeTreeView(QWidget * parent = 0);
    NodeTreeView(bool fixedSize, QWidget * parent = 0);
    void init();
    virtual void registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter);
    QSize sizeHint() const;
    QSize sizeHint(QModelIndex index) const;
    void setModel(QAbstractItemModel *model);

public Q_SLOTS:
    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<AbstractNodeDelegate> delegate);
    void sizeToFit();

private Q_SLOTS:
    void applyFilters();
    void applyFilters(QModelIndex const&);
    bool applyFilters(boost::shared_ptr<Node> const&);
    void toggleExpanded(QModelIndex const&);
    void mouseReleaseEvent(QMouseEvent *event);

Q_SIGNALS:
    void onKeyPressed(QKeyEvent *event);

protected:
    std::vector<boost::shared_ptr<NodeFilterInterface> > m_filters;
    void keyPressEvent(QKeyEvent *event);
    boost::shared_ptr<DelegateProxy> m_delegateMap;
    bool m_fixedSize;
};

} // namespace gui
} // namespace cauv


#endif // __CAUV_NODEPICKER_H__
