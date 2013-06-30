/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __BEARING_RANGE_TO_X_Y_NODE_H__
#define __BEARING_RANGE_TO_X_Y_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <boost/make_shared.hpp>
#include <boost/tuple/tuple.hpp> // for tie()

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
            registerInputID("polar image", Const, May_Be_Old);
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
                debug(4) << "bearingRangeToXY: range" << a.ranges->front() << "--" << a.ranges->back();
                for (KeyPoint const& k : m_polar_keypoints){
                    float range, bearing;
                    cv::Point2f xy = a.xyAt(k.pt.y, k.pt.x, range, bearing);
                    float range_scale = range / a.ranges->back();
                    // TODO: convert angle
                    r.push_back(KeyPoint(
                        floatXY(xy.x,xy.y), k.size*range_scale, 0, k.response, k.octave, k.class_id
                    ));
                }
                return r;
            }
            kp_vec operator()(PyramidMat) const{
                throw parameter_error("image must be polar - it is used for meta information");
            }
            kp_vec m_polar_keypoints;
        };
        void doWork(in_image_map_t& inputs, out_map_t& r){

            image_ptr_t img = inputs["polar image"];
            
            // In a rather special case, we want to propagate the uid from the
            // input keypoints to the output keypoints, NOT have the out_map_t
            // autmagially assign a uid based on the input (an image that we
            // only use for its metadata)
            kp_vec in_kps;
            UID kps_uid;
            boost::tie(in_kps, kps_uid) = paramAndUID< kp_vec >("keypoints");

            debug(4) << "BearingRangeToXYNode:" << in_kps.size() << "kps, img:"
                     << img->id() << "output will have uid:" << kps_uid;
            
            r.internalValue("keypoints") = InternalParamValue(img->apply_visitor(convertKeyPoints(in_kps)), kps_uid);
            
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __BEARING_RANGE_TO_X_Y_NODE_H__

