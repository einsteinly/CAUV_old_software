#ifndef INPUT_NODE_H
#define INPUT_NODE_H

#include "../node.h"

class InputNode: public Node{ 
        typedef boost::lock_guard<boost::recursive_mutex> lock_t;

    public:
        enum Subscription {ImageData, SonarData};
        InputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t),m_subscriptions(),
              ignored(0), processed(0), dropped(0),
              dropped_since(0), m_counters_lock(), m_latest_image_msg(),
              m_latest_sonardata_msg(), m_processed_latest(true),
              m_latest_msg_lock(){
            // don't allow the node to be executed until it has input available
            clearAllowQueue();
        }
        virtual ~InputNode(){
            info() << "~InputNode statistics"
                   << "\n\tignored" << ignored
                   << "\n\tprocessed" <<  processed
                   << "\n\tdropped" <<  dropped; 
        }
        
        /**
         * derived nodes must override this in order to receive inputs of a
         *   particular sort (Images or Sonar data)
         */
        virtual std::set<Subscription> const& subscriptions() const{
            return m_subscriptions;
        }

        /**
         * if this image is from the right source:
         *   take a copy of the image message pointer: store it, and
         *   if m_output_demanded, queue this node for execution
         */
        void onImageMessage(boost::shared_ptr<const ImageMessage> m) throw() {
            lock_t l(m_counters_lock);
            debug() << "Input node received an image";
            if(checkSource(m->image().source(), m->source()) &&
               subscriptions().count(ImageData)){
                lock_t l(m_latest_msg_lock);
                if(!m_processed_latest)
                    dropped_since++;
                m_processed_latest = false;
                m_latest_image_msg = m;
                setAllowQueue();
            }else{
                ignored++;
            }
        }
        
        /**
         * ...
         */
        void onSonarDataMessage(boost::shared_ptr<const SonarDataMessage> m){
            lock_t l(m_counters_lock);
            debug() << "Input node received sonar data";
            if(subscriptions().count(SonarData)){
                lock_t l(m_latest_msg_lock);
                if(!m_processed_latest)
                    dropped_since++;
                m_processed_latest = false;
                m_latest_sonardata_msg = m;
                setAllowQueue();
            }else{
                ignored++;
            }
        }

        /**
         * Input nodes should overload this to set which sources they accept
         * images from (Images only)
         */
        virtual bool checkSource(Image::Source const&, CameraID::e const&) throw(){
            return true;
        }
   
        /* input nodes need to be identified so that onImageMessage() can be
         * efficiently called on only input nodes
         */
        virtual bool isInputNode() throw() { return true; }
    
    protected:
        boost::shared_ptr<const ImageMessage> latestImageMsg(){
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
            
            lock_t m(m_latest_msg_lock);
            return m_latest_image_msg;
        }

        boost::shared_ptr<const SonarDataMessage> latestSonarDataMessage(){
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
            
            lock_t m(m_latest_msg_lock);
            return m_latest_sonardata_msg;
        }
        

        std::set<Subscription> m_subscriptions;

    private:
        int ignored;
        int processed;
        int dropped;
        int dropped_since;
        mutable boost::recursive_mutex m_counters_lock;

        boost::shared_ptr<const ImageMessage> m_latest_image_msg;
        boost::shared_ptr<const SonarDataMessage> m_latest_sonardata_msg;
        bool m_processed_latest;
        mutable boost::recursive_mutex m_latest_msg_lock;
};

#endif // ndef INPUT_NODE_H

