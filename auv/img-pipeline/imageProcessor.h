#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>
#include <map>
#include <set>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <common/msg_classes/image.h>
#include <common/mailbox.h>
#include <utility/ratelimit.h>
#include <utility/foreach.h>
#include <generated/message_observers.h>

#include "pipelineTypes.h"
#include "scheduler.h"
#include "node.h"

namespace cauv{
namespace imgproc{

/**
 * ImageProcessor class manages the sending and receiving of messages, and the
 * corresponding manipulation of the image pipeline graph & it's nodes.
 */
class ImageProcessor: public MessageObserver
{
        typedef boost::shared_ptr<InputNode> input_node_ptr_t;
        typedef boost::shared_ptr<Mailbox> mb_ptr_t;
        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<mutex_t> lock_t;
    public:    
        ImageProcessor(mb_ptr_t mailbox);
       
        void start(std::string const& name);

        /**
         * override MessageObserver functions to take actions on messages
         */
        
        /**
         * Notify all input nodes when we receive something that could be their
         * input. It is up to nodes to filter the source of the image to select
         * their input from others.
         */
        virtual void onImageMessage(ImageMessage_ptr m);
        virtual void onSonarDataMessage(SonarDataMessage_ptr m);
        virtual void onSonarImageMessage(SonarImageMessage_ptr m);
        
        /**
         * These messages describe modifications to the pipeline
         */

        virtual void onAddNodeMessage(AddNodeMessage_ptr m);
        virtual void onRemoveNodeMessage(RemoveNodeMessage_ptr m);
        virtual void onSetNodeParameterMessage(SetNodeParameterMessage_ptr m);
        virtual void onAddArcMessage(AddArcMessage_ptr m);
        virtual void onRemoveArcMessage(RemoveArcMessage_ptr m);
        virtual void onGraphRequestMessage(GraphRequestMessage_ptr m);
        virtual void onForceExecRequestMessage(ForceExecRequestMessage_ptr m);
        virtual void onClearPipelineMessage(ClearPipelineMessage_ptr m);

        /**
         * Pipeline discovery messages - for nodes interested in listing all pipelines
         */

        virtual void onPipelineDiscoveryRequestMessage(PipelineDiscoveryRequestMessage_ptr m);

        /** end MessageObserver functions **/

        void removeNode(node_id const& n);

        /**
         * Use m_mailbox (set by constructor) to send the specified message
         */
        void sendMessage(const boost::shared_ptr<const Message> msg, MessageReliability = RELIABLE_MSG) const;
        void subMessage(Message const& msg, node_id const& node);

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
        template<typename message_T>
        bool _filterMatches(boost::shared_ptr<const message_T> msg){
            lock_t l(m_name_lock);
            if(msg->pipelineName() == m_name)
                return true;
            debug(5) << "filter does not match:" << msg->pipelineName() << "!= "
                     << m_name;
            return false;
        }

        void _addNode(node_ptr_t const& p, node_id const& id) throw();
        void _addNode(node_ptr_t const& p) throw();
        void _removeNode(node_id const& id) throw(id_error);
        
        mutable mutex_t m_nodes_lock;
        std::map<node_id, node_ptr_t> m_nodes;
        std::map<node_ptr_t, node_id> m_nodes_rev;
        std::set<input_node_ptr_t> m_input_nodes;

        Scheduler m_scheduler;

        mutable mutex_t m_name_lock;
        std::string m_name;

        mutable mutex_t m_mailbox_lock;
        mb_ptr_t m_mailbox;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __IMAGEPROCESSOR_H__

