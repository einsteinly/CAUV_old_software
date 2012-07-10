#ifndef __COPY_NODE_MASK_H__
#define __COPY_NODE_MASK_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class CopyNodeMask: public Node{
    public:
        CopyNodeMask(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // two input:
            registerInputID("image", Const);
            registerInputID("mask", Const);
            
            // output:
            registerOutputID(Image_Out_Copied_Name, image_ptr_t());
            
            // no parameters
            // registerParamID<>();
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            
            cv::Mat img = inputs["image"]->mat();
            cv::Mat mask = inputs["mask"]->mat();
            
            try{
                cv::Mat out;
                img.copyTo(out, mask);
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "CopyNodeMask:\n\t"
                        << e.err << "\n\t"
                        << e.func << "," << e.file << ":" << e.line << "\n\t";
            }
            
        }
        
        // Register this node type
        DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __COPY_NODE_MASK_H__

