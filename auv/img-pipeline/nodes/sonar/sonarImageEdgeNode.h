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
            registerParamID<int>("line width", 2, "length to draw white");
        }
        
    protected:
        struct applyFilter: boost::static_visitor<void>{
            applyFilter(int min_amp, int min_width, int line_width)
                : m_min_amp(min_amp),
                  m_min_width(min_width),
                  m_line_width(line_width){
            }
            void operator()(cv::Mat) const{
                throw parameter_error("only polar images are supported");
            }
            void operator()(NonUniformPolarMat a) const{
                const int rows = a.mat.rows;
                const int cols = a.mat.cols;
                
                // apply to each col individually
                for(int col=0; col<cols; col++){
                    int successive = 0; // number of points greater than amplitude
                    int pos = rows-1; //next point to set value at
                    // set all black until we reach success
                    for(int row=rows-1; row>=0; row--){
                        if(a.mat.at<uint8_t>(row, col)>=m_min_amp){
                            successive++;
                            if(successive>=m_min_width){
                                break;
                            }
                        }
                        else{
                            //set all previous to black as well
                            while(pos>=row){
                                a.mat.at<uint8_t>(pos, col) = 0;
                                pos--;
                            }
                            successive = 0;
                        }
                    }
                    //set line width white
                    int w = m_line_width;
                    while(pos>=0 and w>0){
                        a.mat.at<uint8_t>(pos, col) = 255;
                        pos--;
                        w--;
                    }
                    //and remaining black
                    while(pos>=0){
                        a.mat.at<uint8_t>(pos, col) = 0;
                        pos--;
                    }
                }
            }
            void operator()(PyramidMat) const{
                throw parameter_error("only polar images are supported");
            }
            const int m_min_amp;
            const int m_min_width;
            const int m_line_width;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            
            image_ptr_t img = inputs["polar image"];
            
            const int min_amp = param<int>("min amplitude");
            const int min_width = param<int>("min width");
            const int line_width = param<int>("line width");
            
            augmented_mat_t in = img->augmentedMat();

            try{
                boost::apply_visitor(applyFilter(min_amp, min_width, line_width), in);
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