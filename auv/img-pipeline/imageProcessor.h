#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>
#include <map>
#include <set>

#include <boost/shared_ptr.hpp>

#include <common/messages.h>
#include <common/cauv_utils.h>
#include <common/spread/cauv_spread_rc_mailbox.h>

#include "pipelineTypes.h"
#include "scheduler.h"
#include "node.h"

/**
 * ImageProcessor class manages the sending and receiving of messages, and the
 * corresponding manipulation of the image pipeline graph & it's nodes.
 */
class ImageProcessor: public MessageObserver
{
        typedef boost::shared_ptr<InputNode> input_node_ptr_t;
        typedef boost::shared_ptr<ReconnectingSpreadMailbox> mb_ptr_t;
        typedef Spread::service service_t;
    public:    
        ImageProcessor(mb_ptr_t mailbox);
        
        /**
         * override MessageObserver functions to take actions on messages
         */
        
        /**
         * Notify all input nodes when we receive something that could be their
         * input. It is up to nodes to filter the source of the image to select
         * their input from others.
         */
        void onImageMessage(boost::shared_ptr<ImageMessage> m);
        
        /**
         * These messages describe modifications to the pipeline
         */

        void onAddNodeMessage(boost::shared_ptr<AddNodeMessage> m);
        void onRemoveNodeMessage(boost::shared_ptr<RemoveNodeMessage> m);
        void onSetNodeParameterMessage(boost::shared_ptr<SetNodeParameterMessage> m);

        /** end MessageObserver functions **/

        /**
         * Use m_mailbox (set by constructor) to send the specified message
         */
        void sendMessage(Message const& msg, service_t p = SAFE_MESS) const;

        ~ImageProcessor();
    
    private:
        /**
         * Provide a safe id lookup (make sure we don't create NULL nodes)
         */
        node_ptr_t _lookupNode(node_id const& id) const throw(id_error){
            std::map<node_id, node_ptr_t>::const_iterator i = m_nodes.find(id);
            if(i != m_nodes.end())
                return i->second;
            else
                throw(id_error(std::string("Unknown node id") + to_string(id)));
        }

        node_id _newID(node_ptr_t n) const throw(){
            // Can probably do better than this...
            static node_id id = 1;
            return id++;
        }

        std::map<node_id, node_ptr_t> m_nodes;
        std::set<input_node_ptr_t> m_input_nodes;

        Scheduler m_scheduler;
        mb_ptr_t m_mailbox;
};

#endif // ndef __IMAGEPROCESSOR_H__

