/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __LOCAL_MAXIMA_NODE_H__
#define __LOCAL_MAXIMA_NODE_H__

#include <numeric>

#include <boost/make_shared.hpp>

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
            registerInputID("image_in"); 
            registerOutputID("keypoints", std::vector<KeyPoint>());
            registerParamID<float>("delta", 1.0f, "");
        }

    protected:
        struct applyGlobalMaxima: boost::static_visitor< std::vector<KeyPoint> >{
            applyGlobalMaxima(float delta) : m_delta(delta){ }
            std::vector<KeyPoint> operator()(cv::Mat a) const{
                std::vector<KeyPoint> r;
                if(a.channels() != 1)
                    throw parameter_error("image must have 1 channel");
                if((a.type() & CV_MAT_DEPTH_MASK) != CV_8U)
                    throw parameter_error("image must be unsigned bytes");
                
                for(int row = 1; row < a.rows-1; row++)
                    for(int col = 1; col < a.cols-1; col++){
                        const uint8_t v = a.at<uint8_t>(row,col);
                        uint8_t s[8];
                        if(v >= (s[0] = a.at<uint8_t>(row-1,col-1)+m_delta) && 
                           v >= (s[1] = a.at<uint8_t>(row-1,col  )+m_delta) && 
                           v >= (s[2] = a.at<uint8_t>(row-1,col+1)+m_delta) && 
                           v >= (s[3] = a.at<uint8_t>(row,  col-1)+m_delta) && 
                           v >= (s[4] = a.at<uint8_t>(row,  col+1)+m_delta) && 
                           v >= (s[5] = a.at<uint8_t>(row+1,col-1)+m_delta) && 
                           v >= (s[6] = a.at<uint8_t>(row+1,col  )+m_delta) && 
                           v >= (s[7] = a.at<uint8_t>(row+1,col+1)+m_delta)){
                            int surround_mean = std::accumulate(s,s+8,0) / 8 - m_delta;
                            r.push_back(KeyPoint(floatXY(col,row), 3, 0, v - surround_mean, 0, 0));
                        }
                    }
                return r;
            }
            std::vector<KeyPoint> operator()(NonUniformPolarMat a) const{
                std::vector<KeyPoint> r = operator()(a.mat);
                // TODO: should probably leave this conversion to a
                // bearingRangeToXYNode...
                foreach(KeyPoint& k, r){
                    k.pt.x = a.bearings->at(int(k.pt.x));
                    k.pt.y = a.ranges->at(int(k.pt.y));
                }
                return r;
            }
            std::vector<KeyPoint> operator()(PyramidMat) const{
                error() << "local maxima does not support pyramids";
                return std::vector<KeyPoint>();
            }
            float m_delta;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["image_in"];
            const float delta = param<float>("delta");
            
            augmented_mat_t in = img->augmentedMat();

            try{
                r["keypoints"] = NodeParamValue(
                    boost::apply_visitor(applyGlobalMaxima(delta), in)
                );
            }catch(cv::Exception& e){
                error() << "LocalMaximaNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __LOCAL_MAXIMA_NODE_H__


