/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __SUMSQUARED_NODE_H__
#define __SUMSQUARED_NODE_H__

#include <boost/bind.hpp>

#include <opencv2/core/core.hpp>

#include "../node.h"
#include "../nodeFactory.h"
#include "../pipelineTypes.h"


namespace cauv{
namespace imgproc{

class SumSquaredNode: public Node{
    public:
        SumSquaredNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;

            // inputs:
            registerInputID("image", Const);

            // one output
            registerOutputID<float>("sum sq", 0.0f);
        }

    protected:

        static float sumsquared(const cv::Mat& m) {
            if (m.channels() != 1)
                throw parameter_error("Input matrix has to be single channel");

            float sumsq = 0;
            cv::MatConstIterator_<unsigned char> it, itend;
            for (it = m.begin<unsigned char>(), itend = m.end<unsigned char>(); it != itend; ++it)
                sumsq += *it/255.0f;
            
            return sumsq / m.total();
        }

        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["image"];
            r["sum sq"] = img->apply(boost::bind(sumsquared, _1));
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SUMSQUARED_NODE_H__
