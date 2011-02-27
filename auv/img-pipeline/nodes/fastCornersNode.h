#ifndef __FAST_CORNERSNODE_H__
#define __FAST_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class FASTCornersNode: public OutputNode{
    public:
        FASTCornersNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : OutputNode(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID<image_ptr_t>(Image_Out_Copied_Name);
            //registerOutputID<> TODO: KeyPoints output
            
            // parameters:
            registerParamID<int>("threshold", 20, // default value a complete guess
                                 "brightness threshold for contiguous arc of pixels around corner"); 
            registerParamID<bool>("non-maximum suppression", true,
                                  "omit non-maximal corners within 3x3 pixels");
            registerParamID<std::string>("name", "unnamed FAST corners",
                                         "name for detected set of corners");
        }
    
        virtual ~FASTCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const int nonmaxsupp = param<bool>("non-maximum suppression");
            const int threshold = param<int>("threshold");
            const std::string name = param<std::string>("name");

            cv::vector<cv::KeyPoint> corners;
            try{
                cv::FAST(img->cvMat(), corners, threshold, nonmaxsupp);
                
                if(numChildren()){
                    // then produce an output image overlay
                    boost::shared_ptr<Image> out = boost::make_shared<Image>();
                    
                    // make a colour copy to draw pretty corners on
                    cvtColor(img->cvMat(), out->cvMat(), CV_GRAY2BGR);

                    for(cv::vector<cv::KeyPoint>::const_iterator i = corners.begin(); i != corners.end() ; i++)
                        cv::circle(out->cvMat(), i->pt, i->size * i->response, CV_RGB(50, 255, 50), 1, 4);
                    r[Image_Out_Name] = out;
                }
            }catch(cv::Exception& e){
                error() << "FASTCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // convert coordinates from pixels (top left origin) to 0-1 float,
            // top left origin // TODO: check this
            std::vector<Corner> msg_corners;
            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            debug(2) << "FASTCorners: detected" << corners.size() << "corners:";
            for(cv::vector<cv::KeyPoint>::const_iterator i = corners.begin(); i != corners.end() ; i++){
                const floatXYZ centre(i->pt.x / width, i->pt.y / height, 0);
                const Corner c(centre, i->size, i->angle, i->response); 
                debug(3) << c;
                msg_corners.push_back(c);
            }
            sendMessage(boost::make_shared<CornersMessage>(name, msg_corners));

            return r;
        }
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FAST_CORNERSNODE_H__

