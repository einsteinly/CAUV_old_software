/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __TRANSFORM_KEY_POINTS_NODE_H__
#define __TRANSFORM_KEY_POINTS_NODE_H__

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

        void doWork(in_image_map_t&, out_map_t& r){
            std::vector<KeyPoint> kps = param< std::vector<KeyPoint> >("keypoints");
            float px = param<float>("+x");
            float py = param<float>("+y");
            float mx = param<float>("*x");
            float my = param<float>("*y");
            
            std::vector<KeyPoint> transformed = kps;
            for (KeyPoint& t : transformed){
                t.pt.x = t.pt.x*mx + px;
                t.pt.y = t.pt.y*my + py;
            }

            r["keypoints"] = transformed;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __TRANSFORM_KEY_POINTS_NODE_H__


