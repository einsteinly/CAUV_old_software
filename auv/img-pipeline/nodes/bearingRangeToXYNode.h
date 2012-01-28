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

#ifndef __BEARING_RANGE_TO_X_Y_NODE_H__
#define __BEARING_RANGE_TO_X_Y_NODE_H__

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

class BearingRangeToXYNode: public Node{
    public:
        BearingRangeToXYNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // slow node: so that nodes providing input aren't re-scheduled
            // until we're done here - this is more likely to keep them in sync
            m_speed = slow;

            // input polar image:
            registerInputID("polar image", May_Be_Old);
            // param input:
            registerParamID< std::vector<KeyPoint> >(
                "keypoints", std::vector<KeyPoint>(), "the KeyPoints to convert", Must_Be_New
            );
            
            // output
            registerOutputID("keypoints", std::vector<KeyPoint>());
        }

    protected:
        typedef std::vector<KeyPoint> kp_vec;
        /*
        struct KeyPoint
        {
            pt : floatXY;
            size : float;
            angle : float;
            response : float;
            octave : float;
            class_id : float;
        }
        */

        struct convertKeyPoints: boost::static_visitor< kp_vec >{
            convertKeyPoints(kp_vec const& in_kps)
                : m_polar_keypoints(in_kps){
            }
            kp_vec operator()(cv::Mat) const{
                throw parameter_error("image must be polar - it is used for meta information");
            }
            kp_vec operator()(NonUniformPolarMat a) const{
                kp_vec r;
                r.reserve(m_polar_keypoints.size());
                if(a.bearings->size() == 0 || a.ranges->size() == 0)
                    throw parameter_error("invalid polar image: no metadata");
                foreach(KeyPoint const& k, m_polar_keypoints){
                    float bearing_idx = k.pt.x;
                    float range_idx = k.pt.y;
                    // TODO: interpolate?
                    int bearing_idx_r = clamp(0, std::floor(0.5+bearing_idx), a.bearings->size()-1);
                    int range_idx_r = clamp(0, std::floor(0.5+range_idx), a.ranges->size()-1);
                    float bearing = a.bearings->at(bearing_idx_r);
                    float range = a.ranges->at(range_idx_r);
                    float range_scale = range / *(a.ranges->rbegin());
                    // x increases along the zero-bearing axis, y decreases
                    // with increasing bearing in the first quadrant
                    // (mathematical angles)
                    // the bearings are in radians (see SonarInputNode)
                    floatXY xy(range*cos(bearing), -range*sin(bearing));
                    // TODO: convert angle
                    // response value is scaled by range: features further away
                    // are logarithmically more important
                    r.push_back(KeyPoint(
                        xy, k.size*range_scale, 0, k.response*std::log(1+range_scale), k.octave, k.class_id
                    ));
                }
                return r;
            }
            kp_vec operator()(PyramidMat) const{
                throw parameter_error("image must be polar - it is used for meta information");
            }
            kp_vec m_polar_keypoints;
        };
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            image_ptr_t img = inputs["polar image"];
            
            kp_vec in_kps = param< kp_vec >("keypoints");
            augmented_mat_t in = img->augmentedMat();

            r["keypoints"] = ParamValue(boost::apply_visitor(
                convertKeyPoints(in_kps), in
            ));
            
            return r;
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BEARING_RANGE_TO_X_Y_NODE_H__

