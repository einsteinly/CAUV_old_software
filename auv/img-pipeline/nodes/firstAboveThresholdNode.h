/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
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

#ifndef __FIRST_ABOVE_THRESHOLDNODE_H__
#define __FIRST_ABOVE_THRESHOLDNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <boost/math/special_functions/hypot.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <generated/types/Line.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class FirstAboveThresholdNode: public Node{
    public:
        typedef std::vector<floatXY> ret_vec;

        FirstAboveThresholdNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output
            registerOutputID("points", ret_vec());
            
            // parameters:
            registerParamID<int>("threshold", 80);
        }

    protected:
        struct findFirstAboveThreshold: boost::static_visitor< ret_vec >{
            findFirstAboveThreshold(int threshold) : threshold(threshold)
            {
            }

            ret_vec operator()(cv::Mat) const{
                throw parameter_error("image must be polar");
            }
            ret_vec operator()(NonUniformPolarMat a) const{
                ret_vec r;
                cv::Mat_<uchar> m = a.mat;
                
                std::vector<bool> found(m.cols);
                for(int y = 0; y < m.rows; y++)
                {
                    uchar* val = m[y];
                    for(int x = 0; x < m.cols; x++, val++)
                    {
                        if(found[x])
                            continue;

                        if(*val > threshold)
                        {
                            r.push_back(floatXY((*a.bearings)[x], (*a.ranges)[y]));
                            found[x] = true;
                        }
                    }
                } 
                
                return r;
            }
            ret_vec operator()(PyramidMat) const{
                throw parameter_error("image must be polar");
            }

            int threshold;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            augmented_mat_t in = inputs[Image_In_Name]->augmentedMat();
            const int threshold = param<int>("threshold");
            r["points"] = boost::apply_visitor(findFirstAboveThreshold(threshold), in);
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __FIRST_ABOVE_THRESHOLDNODE_H__

