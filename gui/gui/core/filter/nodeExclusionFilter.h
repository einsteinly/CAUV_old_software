/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_NODEEXCLUSIONFILTER_H__
#define __CAUV_NODEEXCLUSIONFILTER_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <filter/nodeFilterInterface.h>

#include <model/nodeType.h>

namespace cauv {
namespace gui {


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


} // namespace gui
} // namespace cauv


#endif // __CAUV_NODEEXCLUSIONFILTER_H__
