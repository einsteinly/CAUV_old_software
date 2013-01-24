/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __SPLIT_HSV_NODE_H__
#define __SPLIT_HSV_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class SplitHSVNode: public Node{
    public:
        SplitHSVNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // input:
            registerInputID("image", Const);
            
            // outputs:
            registerOutputID("H");
            registerOutputID("S");
            registerOutputID("V");
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            cv::Mat img = inputs["image"]->mat();
            cv::Mat HSV;
            int conversion_code = 0;
            
            if(img.channels() == 3)
                conversion_code = CV_BGR2HSV;
            else
                // oops... cvtColor can't do anything else
                throw(parameter_error("image must be 3-channel RGB"));

            try{
                cv::cvtColor(img, HSV, conversion_code, 0);
            }catch(cv::Exception& e){
                error() << "SplitHSVNode (HSV conversion):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
            cv::Mat out[3];

            try{
                cv::split(HSV, out);
            }catch(cv::Exception& e){
                error() << "SplitHSVNode (HSV Split):\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }

            r["H"] = boost::make_shared<Image>(out[0]);
            r["S"] = boost::make_shared<Image>(out[1]);
            r["V"] = boost::make_shared<Image>(out[2]);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SPLIT_HSV_NODE_H__

