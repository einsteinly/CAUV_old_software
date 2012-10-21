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

#ifndef __CAUV_NODEPICKER_H__
#define __CAUV_NODEPICKER_H__

#include <QtGui>

#include <boost/shared_ptr.hpp>
#include <boost/weak_ptr.hpp>

#include <filter/nodeFilterInterface.h>
#include <filter/nodePathFilter.h>

#include <trees/nodeTreeView.h>

#include <model/nodeType.h>

namespace Ui {
class NodePicker;
}

namespace cauv {
namespace gui {

class NodeItemModel;
class AbstractNodeDelegate;

/**
* NodeTreeView with a filter entry box above it
*/
class NodePicker : public QWidget {
    Q_OBJECT

public:
    NodePicker(boost::shared_ptr<NodeItemModel> const& root);
    virtual ~NodePicker();

    void registerDelegate(node_type nodeType,
                          boost::shared_ptr<AbstractNodeDelegate> delegate);
    void registerListFilter(boost::shared_ptr<NodeFilterInterface> const& filter);

public Q_SLOTS:
        void setHighlighting(QString);

protected Q_SLOTS:
    void redirectKeyboardFocus(QKeyEvent* key);

protected:
    boost::shared_ptr<NodeItemModel> m_root;

private:
    Ui::NodePicker *ui;
    QPushButton * m_clearButton;
};

} // namespace gui
} // namespace cauv


#endif // __CAUV_NODEPICKER_H__
