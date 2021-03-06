/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __HOUGH_LINESNODE_H__
#define __HOUGH_LINESNODE_H__

#include <vector>
#include <cmath>

#include <generated/types/Line.h>

#include "../node.h"
#include "../nodeFactory.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class HoughLinesNode: public Node{
    public:
        HoughLinesNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name, Const);
            
            // one output
            registerOutputID("lines", std::vector<Line>());
            
            // parameters:
            registerParamID<bool>("probabilistic", true);
            registerParamID<float>("rho", 1.0);
            registerParamID<float>("theta", M_PI/180);
            registerParamID<int>("threshold", 80);
            // probabilistic only:
            registerParamID<int>("minLineLength", 20);
            registerParamID<int>("maxLineGap", 5);
            // non-probabilistic only:
            registerParamID<int>("srn", 0);
            registerParamID<int>("stn", 0);
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r);

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __HOUGH_LINESNODE_H__

