#ifndef INPUT_NODE_H
#define INPUT_NODE_H

#include "../node.h"

class InputNode: public Node{ 
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        InputNode(Scheduler& sched)
            : Node(sched), ignored(0), processed(0), dropped(0),
            dropped_since(0), m_processed_latest(true){
        }
        virtual ~InputNode(){
            info() << "~InputNode statistics"
                   << "\n\tignored" << ignored
                   << "\n\tprocessed" <<  processed
                   << "\n\tdropped" <<  dropped; 
        }

        /**
         * if this image is from the right source:
         *   take a copy of the image message pointer: store it, and
         *   if m_output_demanded, queue this node for execution
         */
        void onImageMessage(boost::shared_ptr<ImageMessage> m) throw() {
            lock_t l(m_counters_lock);
            debug() << "Input node received an image";
            if(checkSource(m->image().source(), m->source())){ 
                lock_t l(m_latest_image_msg_lock);
                if(!m_processed_latest)
                    dropped_since++;
                m_processed_latest = false;
                m_latest_image_msg = m;
                checkAddSched();
            }else{
                ignored++;
            }
        }

        /**
         * Input nodes should overload this to set which sources they accept
         * images from
         */
        virtual bool checkSource(Image::Source const& s, CameraID const& c) throw() = 0;
   
        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() throw() { return true; }
    
    protected:
        virtual bool allowQueueExec() throw(){
            lock_t m(m_latest_image_msg_lock);
            return !!m_latest_image_msg && !m_processed_latest;
        }

        boost::shared_ptr<ImageMessage> latestImageMsg(){
            lock_t l(m_counters_lock);
            debug() << "Grabbing image";
            if(dropped_since > 0){
                warning() << "Dropped" << dropped_since << "frames since last frame processed";
                dropped += dropped_since;
                dropped_since = 0;
            }
            processed++;
            debug() << "Processed" << processed << "images";
            m_processed_latest = true;
            
            lock_t m(m_latest_image_msg_lock);
            return m_latest_image_msg;
        }

    private:
        int ignored;
        int processed;
        int dropped;
        int dropped_since;
        mutable boost::recursive_mutex m_counters_lock;

        boost::shared_ptr<ImageMessage> m_latest_image_msg;
        bool m_processed_latest;
        mutable boost::recursive_mutex m_latest_image_msg_lock;
};

#endif // ndef INPUT_NODE_H

