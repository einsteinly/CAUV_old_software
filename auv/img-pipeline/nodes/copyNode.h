#ifndef __COPY_NODE_H__
#define __COPY_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


namespace cauv{
namespace imgproc{

class CopyNode: public Node{
    public:
        CopyNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image");
            
            // output:
            registerOutputID<image_ptr_t>("image copy");
            
            // no parameters
            // registerParamID<>();
        }
    
        virtual ~CopyNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            image_ptr_t img = inputs["image"];
            
            try{
                r["image copy"] = image_ptr_t(new Image(*img)); 
            }catch(cv::Exception& e){
                error() << "CopyNode:\n\t"
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

#endif // ndef __COPY_NODE_H__

