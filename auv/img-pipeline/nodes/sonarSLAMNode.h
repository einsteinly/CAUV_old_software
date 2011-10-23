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

#ifndef __SONAR_SLAM_NODE_H__
#define __SONAR_SLAM_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class SonarSLAMNode: public Node{
    public:
        SonarSLAMNode(ConstructArgs const& args) : Node(args){ }
        virtual ~SonarSLAMNode(){ stop(); }

        void init(){
            // slow node (don't schedule nodes providing input until we've
            // finished doing work here)
            m_speed = slow;

            // input (well, parameter input that must be new for the node to be
            // executed):
            registerParamID("keypoints", std::vector<KeyPoint>(), "keypoints used to update map", Must_Be_New);
            
            // outputs:
            //registerOutputID<image_ptr_t>("image_out");
            
            // parameters:
            //registerParamID<float>("some scalar param", 1.0f, "...");
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;


            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SONAR_SLAM_NODE_H__

