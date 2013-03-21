/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_PROCESS_NODE_H__
#define __CAUV_PROCESS_NODE_H__

#include <model/messaging.h>

#include <gui/core/model/node.h>
#include <model/nodes/numericnode.h>
#include <model/nodes/stringnode.h>

#include <boost/shared_ptr.hpp>

#include <generated/types/ProcessControlMessage.h>
#include <generated/types/ProcessCommand.h>

namespace cauv {

    namespace gui {

        class ProcessNode : public BooleanNode {
            Q_OBJECT
            public:
                ProcessNode(const nid_t id);
                virtual ~ProcessNode();
                
                void setStats(float cpu, float mem, unsigned int threads, const std::string& status);
                
            protected:
                boost::shared_ptr<NumericNode<float> > m_cpu;
                boost::shared_ptr<NumericNode<float> > m_mem;
                boost::shared_ptr<NumericNode<unsigned int> > m_threads;
                boost::shared_ptr<StringNode> m_status;
        };
    } // namespace gui
} // namespace cauv
#endif // ndef __CAUV_PROCESS_NODE_H__
