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
        BroadcastLinesNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t){
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

    private:
        static cv::Vec4i rThetaLineToSegment(cv::Vec2f const& l, cv::Size const& s){
            cv::Vec4i r;
            float rho = l[0];
            float theta = l[1];
            double a = std::cos(theta);
            double b = std::sin(theta);
            if(std::fabs(a) < 0.001)
            {
                r[0] = r[2] = cvRound(rho);
                r[1] = 0;
                r[3] = s.height;
            }else if(std::fabs(b) < 0.001)
            {
                r[1] = r[3] = cvRound(rho);
                r[0] = 0;
                r[3] = s.width;
            }
            else
            {
                r[0] = 0;
                r[1] = cvRound(rho/b);
                r[2] = cvRound(rho/a);
                r[3] = 0;
            }
            return r;
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BROADCAST_LINESNODE_H__

