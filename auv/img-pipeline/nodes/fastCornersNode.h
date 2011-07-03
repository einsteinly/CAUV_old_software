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
        FASTCornersNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID< NodeParamValue >("corners");
            registerOutputID< NodeParamValue >("corners (KeyPoint)");
            
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
            corners.reserve(cv_corners.size());
            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            debug(2) << "FASTCorners: detected" << cv_corners.size() << "corners:";
            foreach(const cv::KeyPoint &kp, cv_corners)
                corners.push_back(_corner(kp, width, height));
            r["corners"] = corners;
            
            // thin wrapper... don't want to include cv types in serialisation
            std::vector<cauv::KeyPoint> kps;
            kps.reserve(cv_corners.size());
            foreach(const cv::KeyPoint &kp, cv_corners)
                kps.push_back(_cauvKeyPoint(kp));
            r["corners (KeyPoint)"] = kps;

            return r;
        }
    private:
        static cauv::KeyPoint _cauvKeyPoint(cv::KeyPoint const& kp){
            return cauv::KeyPoint(
                floatXY(kp.pt.x,kp.pt.y), kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            );
        }

        Corner _corner(cv::KeyPoint const& kp, const float w, const float h){
            const floatXYZ centre(kp.pt.x / w, kp.pt.y / h, 0);
            return Corner(centre, kp.size, kp.angle, kp.response);
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FAST_CORNERSNODE_H__

