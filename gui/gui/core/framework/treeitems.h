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

#ifndef TREEITEMS_H
#define TREEITEMS_H

#include <QVariant>
#include <QTreeWidgetItem>
#include <cv.h>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include "../model/nodes/numericnode.h"
#include "../model/nodes/compoundnodes.h"
#include "../model/nodes/stringnode.h"
#include "../model/nodes/groupingnode.h"
#include "../core/model/nodes/imagenode.h"


namespace cauv {
    namespace gui {

        /**
  * A NodeTreeItemBase is used as the base class for one row in the
  * table of data streams.
  *
  * @author Andy Pritchard
  */

        class NodeTreeItemBase : public QObject, public QTreeWidgetItem {
            Q_OBJECT
        public:
            NodeTreeItemBase(boost::shared_ptr<Node> const& node, QTreeWidgetItem * parent = 0);
            virtual ~NodeTreeItemBase(){}
            boost::shared_ptr<Node> getNode();
            virtual bool updateNode(QVariant const&);

        public Q_SLOTS:
            NodeTreeItemBase * addNode(boost::shared_ptr<Node> node);
            void updateValue(const QString value);
        private:
            boost::shared_ptr<Node> m_node;
        };

    } // namespace gui
} // namespace cauv

#endif // TREEITEMS_H
