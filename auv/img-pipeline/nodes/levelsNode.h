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

#ifndef __LEVELS_NODE_H__
#define __LEVELS_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <utility/rounding.h>
#include <common/cauv_utils.h>

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
            registerInputID("image");

            // one output
            registerOutputID("image (not copied)");
            
            // parameters:
            registerParamID<int>("white level", 255);
            registerParamID<int>("black level", 0);
        }

    protected:
        struct applyLevels: boost::static_visitor<void>{
            applyLevels(int white, int black) : m_white(white), m_black(black){}
            void operator()(cv::Mat a) const{
                float scale = 1;
                if(m_black != m_white)
                    scale = 255.0f / (m_white - m_black);
                a = (a - m_black) * scale;
            }
            void operator()(NonUniformPolarMat a) const{
                float scale = 1;
                if(m_black != m_white)
                    scale = 255.0f / (m_white - m_black);
                a.mat = (a.mat - m_black) * scale;
            }
            void operator()(PyramidMat a) const{
                float scale = 1;
                if(m_black != m_white)
                    scale = 255.0f / (m_white - m_black);
                foreach(cv::Mat m, a.levels)
                    m = (m - m_black) * scale;
            }
            int m_white;
            int m_black;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            augmented_mat_t img = inputs["image"]->augmentedMat();
            
            int white_level = param<int>("white level");
            int black_level = param<int>("black level");
            
            boost::apply_visitor(applyLevels(white_level, black_level), img);
            
            r["image (not copied)"] = boost::make_shared<Image>(img);
            
            return r;
        }

    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LEVELS_NODE_H__
