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
                void setReady();
                bool isReady();
                
                void setStats(float cpu, float mem, unsigned int threads, const std::string& status);
                
            protected:
                bool m_ready;
        };
    } // namespace gui
} // namespace cauv
#endif // ndef __CAUV_PROCESS_NODE_H__
