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

#ifndef __SOBEL_NODE_H__
#define __SOBEL_NODE_H__

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

class SobelNode: public Node{
    public:
        SobelNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // one input:
            registerInputID("image_in");
            
            // one output
            registerOutputID("image_out");
            
            // parameters: scale factor, interpolation mode
            registerParamID<int>("x order", 0, "Order of the x derivative");
            registerParamID<int>("y order", 0, "Order of the y derivative");
            registerParamID<int>("aperture size", 3, "Size of the Sobel kernel (-1 for Scharr)");
        }

    protected:
        struct applySobel {
            applySobel(int xorder, int yorder, int size) : xorder(xorder), yorder(yorder), size(size) {}        
            cv::Mat operator()(cv::Mat m) const
            {
                cv::Mat ret;
                cv::Sobel(m, ret, -1, xorder, yorder, size);
                return ret;
            }

            int xorder, yorder;
            int size;
        };
        template<typename F>
        struct applyToMatHelper: boost::static_visitor<augmented_mat_t>{
            applyToMatHelper(F func) : func(func) {
            }
            augmented_mat_t operator()(cv::Mat a) const{
                return func(a);
            }
            augmented_mat_t operator()(NonUniformPolarMat a) const{
                NonUniformPolarMat r;
                r.mat = func(a.mat);
                return r;
            }
            augmented_mat_t operator()(PyramidMat a) const{
                error() << "applyToMat does not support pyramids";
                return a;
            }
            F func;
        };
        template<typename F>
        augmented_mat_t applyToMat(augmented_mat_t& in, F func)
        {
            return boost::apply_visitor(applyToMatHelper<F>(func), in);
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image_in"];
            
            int xorder = param<int>("x order");
            int yorder = param<int>("y order");
            int size = param<int>("aperture size");
            
            augmented_mat_t in = img->augmentedMat();

            try{
                r["image_out"] = boost::make_shared<Image>(applyToMat(in, applySobel(xorder,yorder,size)));
            }catch(cv::Exception& e){
                error() << "SobelNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SOBEL_NODE_H__
