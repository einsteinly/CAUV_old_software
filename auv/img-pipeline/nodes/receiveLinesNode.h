/* Copyright 2011 Cambridge Hydronautics Ltd.
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

#ifndef __RECEIVE_LINES_NODE_H__
#define __RECEIVE_LINES_NODE_H__

#include <vector>
#include <string>

#include <boost/algorithm/string/compare.hpp>

#include <generated/types/LinesMessage.h>

#include "inputNode.h"
#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class ReceiveLinesNode: public InputNode{
    public:
        ReceiveLinesNode(ConstructArgs const& args)
            : InputNode(args),
              ignored(0), processed(0), dropped(0), dropped_since(0), m_counters_lock(),
              m_latest_msg(boost::make_shared<LinesMessage>()), m_processed_latest(true), m_latest_msg_lock() {
        }

        void init(){
            // registerInputID()

            // need to receive LinesMessages
            subMessage(LinesMessage());
            
            // one output:
            registerOutputID("lines", std::vector<Line>());
            
            // one parameter: the source camera
            registerParamID<std::string>("name", "unnamed lines",
                                         "name for detected set of lines");
        }
    
        virtual ~ReceiveLinesNode(){
            info() << "~ReceiveLinesNode statistics"
                   << "\n\tignored" << ignored
                   << "\n\tprocessed" <<  processed
                   << "\n\tdropped" <<  dropped; 
        }

        void onLinesMessage(boost::shared_ptr<const LinesMessage> m){
            debug(4) << "ReceiveLines node received a lines message";
            std::string name = param<std::string>("name");
            if(!boost::equal(m->name(), name)) {
                return;
            }

            lock_t l(m_counters_lock);
            
            { lock_t l(m_latest_msg_lock);
                if(!m_processed_latest)
                    dropped_since++;
                m_processed_latest = false;
                m_latest_msg = m;
            }
            setAllowQueue();
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            debug(4) << "ReceiveLinesNode::doWork";
        
            r["lines"] = latestLinesMsg()->get_lines();
            
            // setAllowQueue() is called by InputNode when a new latestLinesMsg
            // is available
            clearAllowQueue();
        }
        
        boost::shared_ptr<const LinesMessage> latestLinesMsg(){
            lock_t l(m_counters_lock);
            debug(4) << "Grabbing image";
            if(dropped_since > 0){
                debug() << "Dropped" << dropped_since << "messages since last frame processed";
                dropped += dropped_since;
                dropped_since = 0;
            }
            processed++;
            debug(4) << "Processed" << processed << "messages";
            
            { lock_t l(m_latest_msg_lock);
                m_processed_latest = true;
                return m_latest_msg;
            }
        }


    private:
        int ignored;
        int processed;
        int dropped;
        int dropped_since;
        mutable boost::recursive_mutex m_counters_lock;
        
        boost::shared_ptr<const LinesMessage> m_latest_msg;
        bool m_processed_latest;
        mutable boost::recursive_mutex m_latest_msg_lock;
        

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RECEIVE_LINES_NODE_H__

