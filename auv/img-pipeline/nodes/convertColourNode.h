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

#ifndef __CONVERTCOLOUR_NODE_H__
#define __CONVERTCOLOUR_NODE_H__

#include <map>
#include <vector>
#include <string>
#include <algorithm>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class ConvertColourNode: public Node{
    public:
        ConvertColourNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image in", Const);
            
            // one output
            registerOutputID("image out", image_ptr_t());
            
            // parameter:
            registerParamID<std::string>("output format", "grey",
                                         "output format: rgb or grey");
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image in"];
            
            std::string out_fmt = param<std::string>("output format");

            std::transform(out_fmt.begin(), out_fmt.end(), out_fmt.begin(), ::tolower);
            std::remove_if(out_fmt.begin(), out_fmt.end(), ::isspace);
            
            // FIXME: this is all a bit of a mess really, cvtColor can do about
            // a million conversions, most of which will never be needed, but
            // which really this node should expose in some nice easy-to-use
            // way.A
            cv::Mat in = img->mat();
            
            int conversion_code = 0;
            if(out_fmt == "rgb" && in.channels() == 1){
                conversion_code = CV_GRAY2RGB;
            }else if(out_fmt == "bgr" && in.channels() == 1){
                conversion_code = CV_GRAY2BGR;
            }else if(out_fmt == "grey" || out_fmt == "gray"){
                if(in.channels() == 3)
                    conversion_code = CV_BGR2GRAY;
                else if(in.channels() == 4)
                    conversion_code = CV_BGRA2GRAY;
            }else{
                throw parameter_error("Invalid output format: " + out_fmt);
            }
            
            cv::Mat out;
            if(conversion_code != 0){
                try{
                    cv::cvtColor(in, out, conversion_code, 0);
                }catch(cv::Exception& e){
                    error() << "ConvertColourNode:\n\t"
                            << e.err << "\n\t"
                            << "in" << e.func << "," << e.file << ":" << e.line << "\n\t"
                            << "The parameters to this node are:\n\t"
                            << "code = " << param<int>("code") << "\n\t"
                            << "channels = " << param<int>("channels") << "\n\t"
                            << "The inputs to this node are:\n\t"
                            << "*image in = " << in;
                }
            }
            
            r["image out"] = boost::make_shared<Image>(out);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CONVERTCOLOUR_NODE_H__

