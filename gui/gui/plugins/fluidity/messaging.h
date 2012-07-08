/* Copyright 2012 Cambridge Hydronautics Ltd.
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

#ifndef __CAUV_AIMESSAGEGENERATORS_H__
#define __CAUV_AIMESSAGEGENERATORS_H__

#include <QObject>

#include <gui/core/model/messaging.h>

#include <generated/types/PipelineDiscoveryResponseMessage.h>
#include <generated/types/NodeAddedMessage.h>

#include <gui/plugins/ai/conditionnode.h>

namespace cauv {
namespace gui {

class FluidityMessageObserver: public QObject, public MessageObserver{
    Q_OBJECT
    public:
        FluidityMessageObserver(boost::shared_ptr< Node > parent);
        virtual ~FluidityMessageObserver();
        void onPipelineDiscoveryResponseMessage(PipelineDiscoveryResponseMessage_ptr m);
        void onNodeAddedMessage(NodeAddedMessage_ptr m);
        void addPipeline(std::string);
    protected:
        boost::shared_ptr<Node> m_parent;
Q_SIGNALS:
        void discoveryMessageReceieved();
};

} // namespace gui
} // namesapce cauv

#endif // __CAUV_AIMESSAGEGENERATORS_H__

