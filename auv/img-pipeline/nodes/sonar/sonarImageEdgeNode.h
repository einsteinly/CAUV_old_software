#ifndef __SONAR_IMAGE_EDGE_NODE_H__
#define __SONAR_IMAGE_EDGE_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include "../../node.h"

namespace cauv{
namespace imgproc{

class SonarImageEdgeNode : public Node {
    public:
        SonarImageEdgeNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("polar image");
            
            // one output
            registerOutputID("polar image");
            
            // parameters: scale factor, interpolation mode
            registerParamID<int>("min amplitude", 20, "lowest amplitude definately in image");
            registerParamID<int>("min width", 2, "length lowest amplitude needs to be to be counted");
        }
        
    protected:
        struct applyFilter: boost::static_visitor<void>{
            applyFilter(int min_amp, int min_width)
                : m_min_amp(min_amp),
                  m_min_width(min_width){
            }
            void operator()(cv::Mat) const{
                throw parameter_error("only polar images are supported");
            }
            void operator()(NonUniformPolarMat a) const{
                const int rows = a.mat.rows;
                const int cols = a.mat.cols;
                
                // apply to each row individually
                for(int row=0; row<rows; row++){
                    int successive = 0; // number of points in a row greater than amplitude
                    int pos = 0; //next point to set value at
                    // set all black until we reach success
                    for(int col=cols-1; col>=0; col--){
                        if(a.mat.at<uint8_t>(row, col)>=m_min_amp){
                            successive++;
                            if(successive>=m_min_width){
                                break;
                            }
                        }
                        else{
                            //set all previous to black as well
                            while(pos<=col){
                                a.mat.at<uint8_t>(row, pos) = 0;
                                pos++;
                            }
                            successive = 0;
                        }
                    }
                    //set remaining white
                    while(pos<=cols){
                        a.mat.at<uint8_t>(row, pos) = 255;
                        pos++;
                    }
                }
            }
            void operator()(PyramidMat) const{
                throw parameter_error("only polar images are supported");
            }
            const int m_min_amp;
            const int m_min_width;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            image_ptr_t img = inputs["polar image"];
            
            const int min_amp = param<int>("min amplitude");
            const int min_width = param<int>("min width");
            
            augmented_mat_t in = img->augmentedMat();

            try{
                boost::apply_visitor(applyFilter(min_amp, min_width), in);
                r["polar image"] = img;
            }catch(cv::Exception& e){
                error() << "SonarImageEdgeNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            return r;
        }
        
    // Register this node type
    DECLARE_NFR;
};
}
}

#endif