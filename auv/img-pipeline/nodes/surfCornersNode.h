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

#ifndef __SURF_CORNERSNODE_H__
#define __SURF_CORNERSNODE_H__

#include <map>
#include <vector>
#include <string>
#include <cmath>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <generated/types/KeyPoint.h>

#include "../node.h"
#include "outputNode.h"


namespace cauv{
namespace imgproc{

class SURFCornersNode: public Node{
    public:
        SURFCornersNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            // fast node:
            m_speed = fast;
            
            // one input:
            registerInputID(Image_In_Name);
            
            // one output:
            registerOutputID("corners (KeyPoint)", std::vector<KeyPoint>());
            
            // parameters:
            registerParamID<float>("hessian threshold", 100,
                                 "threshold for hessian feature detection");
            registerParamID<int>("octaves", 4,
                                 "number of octaves");
            registerParamID<int>("octave layers", 2,
                                  "number of layers per octave");
        }
    
        virtual ~SURFCornersNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            cv::Mat img = inputs[Image_In_Name]->mat();
            
            const float threshold = param<float>("hessian threshold");
            const int octaves = param<int>("octaves");
            const int octaveLayers = param<int>("octave layers");

            cv::vector<cv::KeyPoint> cv_corners;
            try{
                cv::SURF(threshold,octaves,octaveLayers)(img, cv::Mat(), cv_corners);
            }catch(cv::Exception& e){
                error() << "SURFCornersNode:\n\t"
                        << e.err << "\n\t"
                        << "in" << e.func << "," << e.file << ":" << e.line;
            }
            
            // thin wrapper... don't want to include cv types in serialisation
            std::vector<cauv::KeyPoint> kps;
            kps.reserve(cv_corners.size());
            foreach(const cv::KeyPoint &kp, cv_corners)
                kps.push_back(_cauvKeyPoint(kp));
            r["corners (KeyPoint)"] = kps;

            return r;
        }
    private:
        static cauv::KeyPoint _cauvKeyPoint(cv::KeyPoint const& kp){
            return cauv::KeyPoint(
                floatXY(kp.pt.x,kp.pt.y), kp.size, kp.angle, kp.response, kp.octave, kp.class_id
            );
        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __SURF_CORNERSNODE_H__

