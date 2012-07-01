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

#ifndef __CAUV_NODEPICKER_H__
#define __CAUV_NODEPICKER_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <gui/core/model/nodeFilterInterface.h>
#include <gui/core/model/nodeType.h>

namespace Ui {
class NodePicker;
}

namespace cauv {
namespace gui {

class NodeItemModel;
class NodeDelegate;
class DefaultNodeDelegate;
class Node;

/**
* Node filtering by entering a part of the path
*/
class NodePathFilter : public QObject, public NodeFilterInterface {
    Q_OBJECT
public:
    NodePathFilter(QObject * parent = NULL);

public Q_SLOTS:
    void setText(QString const& string);
    QString getText();
    bool filter(boost::shared_ptr<Node> const& node);

protected:
    QString m_text;
    bool containsText(boost::shared_ptr<Node> const& node);

Q_SIGNALS:
    void filterChanged();
};



class NodeExclusionFilter : public QObject, public NodeFilterInterface {
    Q_OBJECT
public:
    NodeExclusionFilter(QObject * parent = NULL);

public Q_SLOTS:
    virtual bool filter(boost::shared_ptr<Node> const& node);
    void addNode(boost::weak_ptr<Node> node);

Q_SIGNALS:
    void filterChanged();

protected:
    std::vector<boost::weak_ptr<Node> > m_nodes;
};


class NodeChildrenExclusionFilter : public NodeExclusionFilter {
    Q_OBJECT
public:
    NodeChildrenExclusionFilter(QObject * parent = NULL);

public Q_SLOTS:
    virtual bool filter(boost::shared_ptr<Node> const& node);
};


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

    void resizeEvent(QResizeEvent *event);

public Q_SLOTS:
    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<NodeDelegate> delegate);
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
    boost::shared_ptr<DefaultNodeDelegate> m_delegateMap;
    bool m_fixedSize;
};


/**
* NodeTreeView with a filter entry box above it
*/
class NodePicker : public QWidget {
    Q_OBJECT

public:
    NodePicker(boost::shared_ptr<NodeItemModel> const& root);
    virtual ~NodePicker();

    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<NodeDelegate> delegate);
    void registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter);

protected Q_SLOTS:
    void redirectKeyboardFocus(QKeyEvent* key);

protected:
    boost::shared_ptr<NodeItemModel> m_root;

private:
    Ui::NodePicker *ui;
};

} // namespace gui
} // namespace cauv


#endif // __CAUV_NODEPICKER_H__
