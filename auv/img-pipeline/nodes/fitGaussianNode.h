#ifndef __FITGAUSSIAN_H__
#define __FITGAUSSIAN_H__

#include <vector>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <generated/types/Ellipse.h>

#include "../node.h"
#include "../nodeFactory.h"
#include "../pipelineTypes.h"

namespace cauv{
namespace imgproc{

class FitGaussianNode : public Node{
    public:
        FitGaussianNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            //Fast node
            m_speed = fast;
            
            registerInputID("image");
            
            registerOutputID("ellipse", std::vector<Ellipse>());
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            cv::Mat img = inputs["image"]->mat();

            if(img.channels() > 1)
                throw(parameter_error("Image must have only one channel."));

            cv::Moments moments = cv::moments(img);
    
            if (moments.m00 == 0)
                return;
            
            double x = moments.m10 / moments.m00;
            double y = moments.m01 / moments.m00;
            double v11 = moments.mu11 / moments.m00;
            double v20 = moments.mu20 / moments.m00;
            double v02 = moments.mu02 / moments.m00;
           
            using std::atan2;
            using std::sqrt;

            double angle = 0.5 * atan2(2*v11, v20-v02);
            double a = sqrt((v20+v02)/2 + sqrt(4*v11*v11 + (v20-v02)*(v20-v02))/2);
            double b = sqrt((v20+v02)/2 - sqrt(4*v11*v11 + (v20-v02)*(v20-v02))/2);

            std::vector<Ellipse> ret;
            ret.push_back(Ellipse(floatXY(x,y),a,b,angle));
            r["ellipse"] = ret;

            return;
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __FITGAUSSIAN_H__
