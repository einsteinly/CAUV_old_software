#ifndef __PYRAMID_NODE_H__
#define __PYRAMID_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"
#include "../nodeFactory.h"


//Used for cropping (self-explanatory) and anti-cropping (ask James Crosby)

namespace cauv{
namespace imgproc{

class PyramidNode: public Node{
    public:
        PyramidNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID(Image_In_Name);
            
            // output:
            registerOutputID<image_ptr_t>(Image_Out_Copied_Name);
            
            // multiple parameters
            registerParamID<int>("level",1,"positive level will call pyrdown, negative will call pyrup");
        }
    
        virtual ~PyramidNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            try{
                int level = param<int>("level");
            
                image_ptr_t img = inputs[Image_In_Name];

                cv::Mat out_mat = img->cvMat();
                cv::Mat tmp;
                if (level < 0)
                {
                    for (int i = 0; i < -level; ++i)
                    {
                        cv::pyrUp(out_mat, tmp);
                        out_mat = tmp;
                    }
                }
                else
                {
                    for (int i = 0; i < level; ++i)
                    {
                        cv::pyrDown(out_mat, tmp);
                        out_mat = tmp;
                    }
                }
                
                r[Image_Out_Copied_Name] = boost::make_shared<Image>(out_mat);
        
            } catch(cv::Exception& e) {
                error() << "PyramidNode:\n\t"
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

#endif // ndef __PYRAMID_NODE_H__

