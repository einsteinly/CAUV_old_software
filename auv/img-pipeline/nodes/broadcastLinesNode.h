#ifndef __BROADCAST_LINESNODE_H__
#define __BROADCAST_LINESNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class BroadcastLinesNode: public OutputNode{
    public:
        BroadcastLinesNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            // slow node:
            m_speed = fast;
            
            // no inputs:
            
            // no outputs
            
            // parameters:
            registerParamID< std::vector<Line> >("lines", std::vector<Line>());
            registerParamID<std::string>("name", "unnamed lines",
                                         "name for detected set of lines");
        }
    
        virtual ~BroadcastLinesNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;

            const std::string name = param<std::string>("name");
            const std::vector<Line> lines = param< std::vector<Line> >("lines");

            sendMessage(boost::make_shared<LinesMessage>(name, lines));

            return r;
        }
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_LINESNODE_H__

