#ifndef __SHITOMASI_CORNERSNODE_H__
#define __SHITOMASI_CORNERSNODE_H__

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

class ShiTomasiCornersNode: public Node{
    public:
        ShiTomasiCornersNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID< NodeParamValue >("corners");
            
            // parameters:
            registerParamID<int>("maxCorners", 100,
                                 "The maximum number of corners to return. If there are more corners than that will be found, the strongest of them will be returned");
            registerParamID<float>("qualityLevel", 0.01,
                                   "Characterizes the minimal accepted quality of image corners; the value of the parameter is multiplied by the by the best corner quality measure (which is the min eigenvalue, see cornerMinEigenVal() , or the Harris function response, see cornerHarris() ). The corners, which quality measure is less than the product, will be rejected. For example, if the best corner has the quality measure = 1500, and the qualityLevel=0.01 , then all the corners which quality measure is less than 15 will be rejected.");
            registerParamID<float>("minDistance", 10,
                                   "The minimum possible Euclidean distance between the returned corners");
            registerParamID<int>("blockSize", 3,
                                 "Size of the averaging block for computing derivative covariation matrix over each pixel neighborhood, see cornerEigenValsAndVecs()");
            registerParamID<bool>("useHarrisDetector", false,
                                  "Indicates, whether to use operator or cornerMinEigenVal()");
            registerParamID<float>("k", 0.04,
                                   "Free parameter of Harris detector");
        }
    
        virtual ~ShiTomasiCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs[Image_In_Name];
            
            const int maxCorners = param<int>("maxCorners");
            const float qualityLevel = param<float>("qualityLevel");
            const float minDistance = param<float>("minDistance");
            const int blockSize = param<int>("blockSize");
            const bool useHarrisDetector = param<bool>("useHarrisDetector");
            const float k = param<float>("k");

            cv::vector<cv::Point2f> cv_corners;
            try{
                cv::goodFeaturesToTrack(img->cvMat(),
                                        cv_corners,
                                        maxCorners,
                                        qualityLevel,
                                        minDistance,
                                        cv::Mat(),
                                        blockSize,
                                        useHarrisDetector,
                                        k);
            }catch(cv::Exception& e){
                error() << "ShiTomasiCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // convert coordinates from pixels (top left origin) to 0-1 float,
            // top left origin // TODO: check this
            std::vector<Corner> corners;
            const float width = img->cvMat().cols;
            const float height = img->cvMat().rows;
            debug(2) << "ShiTomasiCorners: detected" << cv_corners.size() << "corners:";
            foreach(const cv::Point2f &p, cv_corners) {
                const floatXYZ centre(p.x / width, p.y / height, 0);
                const Corner c(centre, blockSize, 0, 1); 
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

#endif // ndef __SHITOMASI_CORNERSNODE_H__

