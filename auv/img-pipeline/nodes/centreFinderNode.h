#ifndef __CENTREFINDER_H__
#define __CENTREFINDER_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <generated/messages.h>

#include "../node.h"
#include "outputNode.h"

namespace cauv{
namespace imgproc{

class CentreFinderNode : public OutputNode{
    public:
        CentreFinderNode(ConstructArgs const& args)
            : OutputNode(args){
        }

        void init(){
            //Fast node
            m_speed = fast;
            
            //One input
            registerInputID("image_in");
            
            //No output
            
            //Parameters
            registerParamID<std::string>("name", "unnamed region centre"
                                         "name for detected centre");
            
        }
    
        virtual ~CentreFinderNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            std::string name = param<std::string>("name");

            image_ptr_t img = inputs["image_in"];

            if(!img->cvMat().isContinuous())
                throw(parameter_error("Image must be continuous."));
            if((img->cvMat().type() & CV_MAT_DEPTH_MASK) != CV_8U)
                throw(parameter_error("Image must have unsigned bytes."));
            if(img->cvMat().channels() > 1)
                throw(parameter_error("Image must have only one channel."));
                //TODO: support vector parameters

            int totalX = 0;
            int totalY = 0;
            int sum = 0;

            for(int i = 0; i < img->cvMat().cols; i++){
                for(int j = 0; j < img->cvMat().rows; j++){
                    if(img->cvMat().at<uint8_t>(j, i) > 127){
                        totalX += i;
                        totalY += j;
                        sum++;
                    }
                }
            }

            float x = 0;
            float y = 0;
            if(sum == 0){
                x = 0;
                y = 0;
            } else {
                x = ((float) totalX) / ((float) sum);
                y = ((float) totalY) / ((float) sum);
            }
            sendMessage(boost::make_shared<CentreMessage>(name, x / img->cvMat().cols, y / img->cvMat().rows));
            return r;
        }

    //Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif //ndef __CENTREFINDER_H__
