/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __RENDER_STRING_NODE_H__
#define __RENDER_STRING_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../node.h"


namespace cauv{
namespace imgproc{

class RenderStringNode: public Node{
    public:
        RenderStringNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            registerInputID("image_in", NonConst);
            registerOutputID("image_out (not copied)"); 
            registerParamID<std::string>("string", std::string(), "string to render");
            registerParamID<int>("position: x", 0, "");
            registerParamID<int>("position: y", 0, "");
            registerParamID<float>("size", 1, "");
        }

    protected:
        struct applyRenderString: boost::static_visitor<augmented_mat_t>{
            applyRenderString(int x, int y, float sz, std::string const& s)
                : m_x(x), m_y(y), m_sz(sz), m_str(s){
            }
            augmented_mat_t operator()(cv::Mat a) const{
                cv::putText(a, m_str, cv::Point(m_x, m_y),
                cv::FONT_HERSHEY_PLAIN, m_sz, cv::Scalar::all(255), 1, 8);
                return a;
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                error() << "render string does not support polar images";
                return a;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "render string does not support pyramids";
                return a;
            }
            int m_x;
            int m_y;
            float m_sz;
            std::string const& m_str;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            
            std::string str = param<std::string>("string");
            int x = param<int>("position: x");
            int y = param<int>("position: y");
            float s = param<float>("size");
            
            augmented_mat_t in = img->augmentedMat();
            augmented_mat_t out;

            try{
                out = boost::apply_visitor(applyRenderString(x, y, s, str), in);
                r["image_out (not copied)"] = boost::make_shared<Image>(out);
            }catch(cv::Exception& e){
                error() << "RenderStringNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RENDER_STRING_NODE_H__

