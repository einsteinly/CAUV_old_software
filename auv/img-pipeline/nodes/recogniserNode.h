#ifndef __RECOGNISER_NODE_H__
#define __RECOGNISER_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class RecogniserNode: public Node{
    public:
        RecogniserNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;

            // input:
            registerParamID< std::vector<KeyPoint> >("Reference KeyPoints", std::vector<KeyPoint>(), "");
            registerParamID< std::vector<KeyPoint> >("Image KeyPoints", std::vector<KeyPoint>(), "", Must_Be_New);

            // outputs:
            registerOutputID<NodeParamValue>("Confidence");
            registerOutputID<NodeParamValue>("Position");
        }

        virtual ~RecogniserNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;

            std::vector<KeyPoint> ref_kps = param< std::vector<KeyPoint> >("Reference KeyPoints");
            m_reference_keypoints = conv(ref_kps);
            
            std::vector<cv::KeyPoint> img_kps = _cvKeyPointVec(param< std::vector<KeyPoint> >("Image KeyPoints"));

            return r;
        }
        
        std::vector<cv::KeyPoint> _cvKeyPointVec(std::vector<KeyPoint> const& v){
            std::vector<cv::KeyPoint> r;
            r.reserve(v.size());
            std::vector<KeyPoint>::const_iterator i;
            for(i = v.begin(); i != v.end(); i++)
                r.push_back(_cvKeyPoint(*i));
            return r;
        }
    
    private:
        static cv::KeyPoint _cvKeyPoint(cauv::KeyPoint const& kp) const{ 
            return cv::KeyPoint(kp.pt.x, kp.pt.y, kp.size, kp.angle,
                                kp.response, kp.octave, kp.class_id);
        }

        std::vector<cv::KeyPoint> m_reference_keypoints;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __RECOGNISER_NODE_H__


