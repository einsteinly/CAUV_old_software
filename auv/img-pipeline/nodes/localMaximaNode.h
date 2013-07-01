/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LOCAL_MAXIMA_NODE_H__
#define __LOCAL_MAXIMA_NODE_H__

#include <numeric>

#include <boost/make_shared.hpp>
#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class LocalMaximaNode: public Node{
    public:
        LocalMaximaNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;
            registerInputID("image_in", Const); 
            registerOutputID("keypoints", std::vector<KeyPoint>());
            registerParamID<float>("delta", 1.0f, "");
        }

    protected:
        static std::vector<KeyPoint> globalMaxima(cv::Mat a, float delta) {
            std::vector<KeyPoint> r;
            if(a.channels() != 1)
                throw parameter_error("image must have 1 channel");
            if(a.type() != CV_8U)
                throw parameter_error("image must be unsigned bytes");
            
            for(int row = 1; row < a.rows-1; row++)
                for(int col = 1; col < a.cols-1; col++){
                    const uint8_t v = a.at<uint8_t>(row,col);
                    uint8_t s[8];
                    if(v >= (s[0] = a.at<uint8_t>(row-1,col-1)+delta) && 
                       v >= (s[1] = a.at<uint8_t>(row-1,col  )+delta) && 
                       v >= (s[2] = a.at<uint8_t>(row-1,col+1)+delta) && 
                       v >= (s[3] = a.at<uint8_t>(row,  col-1)+delta) && 
                       v >= (s[4] = a.at<uint8_t>(row,  col+1)+delta) && 
                       v >= (s[5] = a.at<uint8_t>(row+1,col-1)+delta) && 
                       v >= (s[6] = a.at<uint8_t>(row+1,col  )+delta) && 
                       v >= (s[7] = a.at<uint8_t>(row+1,col+1)+delta)){
                        int surround_mean = std::accumulate(s,s+8,0) / 8 - delta;
                        r.push_back(KeyPoint(floatXY(col,row), 3, 0, v - surround_mean, 0, 0));
                    }
                }
            return r;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            const float delta = param<float>("delta");
            
            try{
                r["keypoints"] = img->apply(boost::bind(globalMaxima, _1, delta));
            }catch(cv::Exception& e){
                error() << "LocalMaximaNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LOCAL_MAXIMA_NODE_H__


