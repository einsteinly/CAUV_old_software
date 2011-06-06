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
            registerInputID("image in");
            
            // one output
            registerOutputID<image_ptr_t>("image out");
            
            // parameter:
            registerParamID<std::string>("output format", "grey",
                                         "output format: rgb or grey");
        }
    
        virtual ~ConvertColourNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image in"];
            
            std::string out_fmt = param<std::string>("output format");

            std::transform(out_fmt.begin(), out_fmt.end(), out_fmt.begin(), ::tolower);
            std::remove_if(out_fmt.begin(), out_fmt.end(), ::isspace);
            
            // FIXME: this is all a bit of a mess really, cvtColor can do about
            // a million conversions, most of which will never be needed, but
            // which really this node should expose in some nice easy-to-use
            // way.

            int conversion_code = 0;
            if(out_fmt == "rgb" && img->cvMat().channels() == 1){
                conversion_code = CV_GRAY2RGB;
            }else if(out_fmt == "grey" || out_fmt == "gray"){
                if(img->cvMat().channels() == 3)
                    conversion_code = CV_RGB2GRAY;
                else if(img->cvMat().channels() == 4)
                    conversion_code = CV_RGBA2GRAY;
            }else{
                throw parameter_error("Invalid output format: " + out_fmt);
            }
            
            boost::shared_ptr<Image> out = boost::make_shared<Image>();
            if(conversion_code != 0){
                try{
                    cv::cvtColor(img->cvMat(), out->cvMat(), conversion_code, 0);
                }catch(cv::Exception& e){
                    error() << "ConvertColourNode:\n\t"
                            << e.err << "\n\t"
                            << "in" << e.func << "," << e.file << ":" << e.line << "\n\t"
                            << "The parameters to this node are:\n\t"
                            << "code = " << param<int>("code") << "\n\t"
                            << "channels = " << param<int>("channels") << "\n\t"
                            << "The inputs to this node are:\n\t"
                            << "*image in = " << *img;
                }
            }
            
            r["image out"] = out;
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CONVERTCOLOUR_NODE_H__

