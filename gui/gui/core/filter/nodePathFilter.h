/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_NODEPATHFILTER_H__
#define __CAUV_NODEPATHFILTER_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <filter/nodeFilterInterface.h>

#include <model/nodeType.h>

namespace cauv {
namespace gui {

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

} // namespace gui
} // namespace cauv


#endif // __CAUV_NODEPATHFILTER_H__
