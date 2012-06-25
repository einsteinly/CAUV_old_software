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

#ifndef __COPY_NODE_H__
#define __COPY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class CopyNode: public Node{
    public:
        CopyNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");
            
            // output:
            registerOutputID("image copy", image_ptr_t());
            
            // no parameters
            // registerParamID<>();
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            image_ptr_t img = inputs["image"];
            
            try{
                r["image copy"] = boost::make_shared<Image>(*img); 
            }catch(cv::Exception& e){
                error() << "CopyNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
        }
        
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COPY_NODE_H__

