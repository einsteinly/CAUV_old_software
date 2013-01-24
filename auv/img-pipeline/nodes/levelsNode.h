/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LEVELS_NODE_H__
#define __LEVELS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include <utility/rounding.h>
#include <common/msg_classes/colour.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class LevelsNode: public Node{
    public:
        LevelsNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image", NonConst);

            // one output
            registerOutputID("image (not copied)");
            
            // parameters:
            registerParamID<Colour>("black level", Colour::fromGrey(0.0));
            registerParamID<Colour>("white level", Colour::fromGrey(1.0));
        }

    protected:
        static void doLevels(cv::Mat& a, float black, float white) {
            if (black < white) {
                float scale = 1.0f / (white - black);
                a = (a - 255.0f*black) * scale;
            }
            else
                a = cv::Mat(a.size(), a.type(), 255.0f*black);
        }
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            
            float black_level = param<Colour>("black level").grey();
            float white_level = param<Colour>("white level").grey();
            
            img->apply(boost::bind(doLevels, _1, black_level, white_level));
            
            r["image (not copied)"] = img;
            
        }

    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LEVELS_NODE_H__
