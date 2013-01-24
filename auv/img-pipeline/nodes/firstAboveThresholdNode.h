/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
            registerInputID(Image_In_Name, Const);
            
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
                cv::Mat_<uchar> m = a.mat;
                ret_vec r;
                r.reserve(m.cols);
                
                //for each column, work down thorugh until find high enough threshold
                for(int x = 0; x < m.cols; x++)
                {
                    for(int y = 0; y < m.rows; y++)
                    {
                        if(m.at<uint8_t>(y,x) > threshold)
                        {
                            r.push_back(floatXY((*a.bearings)[x],(*a.ranges)[y]));
                            goto next_loop;
                        }
                    }
                    r.push_back(floatXY((*a.bearings)[x],(*a.ranges)[m.rows-1]));
                    next_loop:
                    ;
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

