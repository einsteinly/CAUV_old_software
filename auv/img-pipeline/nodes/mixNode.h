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

#ifndef __MIX_NODE_H__
#define __MIX_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <common/cauv_utils.h>

#include "../node.h"


namespace cauv{
namespace imgproc{

class MixNode: public Node{
    public:
        MixNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image");
            registerInputID("mix");

            // one output
            registerOutputID<image_ptr_t>("image (not copied)");
            
            // parameters:
            registerParamID<float>("image fac", 1);
            registerParamID<float>("mix fac", 1);
            registerParamID<bool>("absolute value", true, "take absolute value "
                "before clamping into pixel range");
        }
    
        virtual ~MixNode(){
            stop();
        }

    protected:
        static unsigned absRound(const float& f){
            if(f > 0)
                return unsigned(f + 0.5);
            else
                return unsigned(0.5 - f);
        }

        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            cv::Mat img = inputs["image"]->mat();
            cv::Mat mix = inputs["mix"]->mat();
            
            float img_f = param<float>("image fac");
            float mix_f = param<float>("mix fac");
            bool do_abs = param<bool>("absolute value");
            
            try {
                img = img*img_f + mix*mix_f;
                if (do_abs)
                    img = cv::abs(img);
                r["image (not copied)"] = boost::make_shared<Image>(img);
            } catch (cv::Exception& e) {
                error() << "MixNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MIX_NODE_H__
