#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>
#include <map>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

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
        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;
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
        virtual void onImageMessage(ImageMessage_ptr m);
        
        /**
         * These messages describe modifications to the pipeline
         */

        virtual void onAddNodeMessage(AddNodeMessage_ptr m);
        virtual void onRemoveNodeMessage(RemoveNodeMessage_ptr m);
        virtual void onSetNodeParameterMessage(SetNodeParameterMessage_ptr m);
        virtual void onAddArcMessage(AddArcMessage_ptr m);
        virtual void onGraphRequestMessage(GraphRequestMessage_ptr m);

        /** end MessageObserver functions **/

        /**
         * Use m_mailbox (set by constructor) to send the specified message
         */
        void sendMessage(const boost::shared_ptr<const Message> msg, service_t p = SAFE_MESS) const;

        ~ImageProcessor();
    
        /**
         * Provide a safe id lookup (make sure we don't create NULL nodes)
         */
        node_ptr_t lookup(node_id const& id) const throw(id_error);
        
        /**
         * and a safe reverse lookup: 0 is returned for non-existent nodes
         */
        node_id lookup(node_ptr_t const& p) const throw();

    private:
        void _addNode(node_ptr_t const& p, node_id const& id) throw();
        void _addNode(node_ptr_t const& p) throw();
        void _removeNode(node_id const& id) throw(id_error);
        node_id _newID(node_ptr_t) const throw();
        
        mutable mutex_t m_nodes_lock;
        std::map<node_id, node_ptr_t> m_nodes;
        std::map<node_ptr_t, node_id> m_nodes_rev;
        std::set<input_node_ptr_t> m_input_nodes;

        Scheduler m_scheduler;

        mutable mutex_t m_mailbox_lock;
        mb_ptr_t m_mailbox;
};

#endif // ndef __IMAGEPROCESSOR_H__

