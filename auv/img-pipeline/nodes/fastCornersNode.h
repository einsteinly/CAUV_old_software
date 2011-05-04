#ifndef __FAST_CORNERSNODE_H__
#define __FAST_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class FASTCornersNode: public Node{
    public:
        FASTCornersNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID< NodeParamValue >("corners");
            
            // parameters:
            registerParamID<int>("threshold", 20, // default value a complete guess
                                 "brightness threshold for contiguous arc of pixels around corner"); 
            registerParamID<bool>("non-maximum suppression", true,
                                  "omit non-maximal corners within 3x3 pixels");
        }
    
        virtual ~FASTCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const bool nonmaxsupp = param<bool>("non-maximum suppression");
            const int threshold = param<int>("threshold");

            cv::vector<cv::KeyPoint> cv_corners;
            try{
                cv::FAST(img->cvMat(), cv_corners, threshold, nonmaxsupp);
            }catch(cv::Exception& e){
                error() << "FASTCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // convert coordinates from pixels (top left origin) to 0-1 float,
            // top left origin // TODO: check this
            std::vector<Corner> corners;
            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            debug(2) << "FASTCorners: detected" << cv_corners.size() << "corners:";
            foreach(const cv::KeyPoint &kp, cv_corners) {
                const floatXYZ centre(kp.pt.x / width, kp.pt.y / height, 0);
                const Corner c(centre, kp.size, kp.angle, kp.response); 
                debug(6) << c;
                corners.push_back(c);
            }
            r["corners"] = corners;

            return r;
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FAST_CORNERSNODE_H__

