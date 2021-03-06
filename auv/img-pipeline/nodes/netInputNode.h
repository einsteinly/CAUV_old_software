/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __NET_INPUT_NODE_H__
#define __NET_INPUT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/types/CameraID.h>
#include <generated/types/ImageMessage.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class NetInputNode: public InputNode{
    public:
        NetInputNode(ConstructArgs const& args)
            : InputNode(args),
              ignored(0), processed(0), dropped(0), dropped_since(0), m_counters_lock(),
              m_latest_image_msg(boost::make_shared<ImageMessage>()), m_processed_latest(true), m_latest_msg_lock() {
        }

        void init(){
            // registerInputID()

            // need to receive ImageMessages
            subMessage(ImageMessage());
            
            // one output:
            registerOutputID("image_out");
            
            // one parameter: the source camera
            registerParamID<int>("camera id", CameraID::Forward);
        }
    
        virtual ~NetInputNode(){
            info() << "~NetInputNode statistics"
                   << "\n\tignored" << ignored
                   << "\n\tprocessed" <<  processed
                   << "\n\tdropped" <<  dropped; 
        }

        /**
         * if this image is from the right source:
         *   take a copy of the image message pointer: store it, and
         *   if m_output_demanded, queue this node for execution
         */
        void onImageMessage(boost::shared_ptr<const ImageMessage> m){
            debug(4) << "Input node received an image";
            if(m->source() != param<int>("camera id")) {
                return;
            }

            lock_t l(m_counters_lock);
            
            { lock_t l(m_latest_msg_lock);
                if(!m_processed_latest)
                    dropped_since++;
                m_processed_latest = false;
                m_latest_image_msg = m;
            }
            setAllowQueue();
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "NetInputNode::doWork";
        
            boost::shared_ptr<Image> out_image = boost::make_shared<Image>();
            latestImageMsg()->get_image_inplace(*out_image);
            r["image_out"] = out_image;
            
            // setAllowQueue() is called by InputNode when a new latestImageMsg
            // is available
            clearAllowQueue();
        }
        
        boost::shared_ptr<const ImageMessage> latestImageMsg(){
            lock_t l(m_counters_lock);
            debug(4) << "Grabbing image";
            if(dropped_since > 0){
                debug() << "Dropped" << dropped_since << "frames since last frame processed";
                dropped += dropped_since;
                dropped_since = 0;
            }
            processed++;
            debug(4) << "Processed" << processed << "images";
            
            { lock_t l(m_latest_msg_lock);
                m_processed_latest = true;
                return m_latest_image_msg;
            }
        }


    private:
        int ignored;
        int processed;
        int dropped;
        int dropped_since;
        mutable boost::recursive_mutex m_counters_lock;
        
        boost::shared_ptr<const ImageMessage> m_latest_image_msg;
        bool m_processed_latest;
        mutable boost::recursive_mutex m_latest_msg_lock;
        

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __NET_INPUT_NODE_H__

