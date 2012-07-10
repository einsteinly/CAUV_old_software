/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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
            registerInputID("image", NonCnst);

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
