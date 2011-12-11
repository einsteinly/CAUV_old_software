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

#ifndef __TRANSFORM_KEY_POINTS_NODE_H__
#define __TRANSFORM_KEY_POINTS_NODE_H__


#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class TransformKeyPointsNode: public Node{
    public:
        TransformKeyPointsNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;
            
            registerOutputID("keypoints", std::vector<KeyPoint>());

            // parameters:
            registerParamID("keypoints", std::vector<KeyPoint>(), "keypoints to transform", Must_Be_New);
            registerParamID("+x", 0.0f);
            registerParamID("+y", 0.0f);
            registerParamID("*x", 1.0f);
            registerParamID("*y", 1.0f);
        }

    protected:

        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            std::vector<KeyPoint> kps = param< std::vector<KeyPoint> >("keypoints");
            float px = param<float>("+x");
            float py = param<float>("+y");
            float mx = param<float>("*x");
            float my = param<float>("*y");
            
            std::vector<KeyPoint> transformed = kps;
            foreach(KeyPoint& t, transformed){
                t.pt.x = t.pt.x*mx + px;
                t.pt.y = t.pt.y*my + py;
            }

            r["keypoints"] = transformed;

            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __TRANSFORM_KEY_POINTS_NODE_H__


