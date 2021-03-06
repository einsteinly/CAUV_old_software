/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __SPLIT_RGB_NODE_H__
#define __SPLIT_RGB_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class SplitRGBNode: public Node{
    public:
        SplitRGBNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // input:
            registerInputID("image", Const);
            
            // outputs:
            registerOutputID("R");
            registerOutputID("G");
            registerOutputID("B");
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            image_ptr_t img = inputs["image"];
            
            cv::Mat out[3];

            try{
                cv::split(img->mat(), out);
            }catch(cv::Exception& e){
                error() << "SplitRGBNode:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["R"] = boost::make_shared<Image>(out[2]);
            r["G"] = boost::make_shared<Image>(out[1]);
            r["B"] = boost::make_shared<Image>(out[0]);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SPLIT_RGB_NODE_H__

