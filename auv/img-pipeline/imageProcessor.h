#ifndef __IMAGEPROCESSOR_H__
#define __IMAGEPROCESSOR_H__

#include <exception>
#include <vector>
#include <stdexcept>
#include <map>
#include <set>

#include <boost/shared_ptr.hpp>

#include <common/messages.h>

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

    public:    
        ImageProcessor();
        
        /**
         * override MessageObserver functions to take actions on messages
         */
        
        /**
         * Notify all input nodes when we receive something that could be their
         * input. It is up to nodes to filter the source of the image to select
         * their input from others.
         */
        void onImageMessage(ImageMessage const& m);
        
        /**
         * These messages describe modifications to the pipeline
         */

        void onAddNodeMessage(AddNodeMessage const& m);
        void onRemoveNodeMessage(RemoveNodeMessage const& m);
        void onSetNodeParameterMessage(SetNodeParameterMessage const& m);

        /** end MessageObserver functions **/

        ~ImageProcessor();
    
    private:
        std::map<node_id, node_ptr_t> m_nodes;
        std::set<input_node_ptr_t> m_input_nodes;

        Scheduler m_scheduler;
};

#endif // ndef __IMAGEPROCESSOR_H__

