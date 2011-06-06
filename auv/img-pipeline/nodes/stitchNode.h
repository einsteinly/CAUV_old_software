#ifndef __STITCH_NODE_H__
#define __STITCH_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class StitchNode: public Node{
    public:
        StitchNode(Scheduler& sched, ImageProcessor& pl, std::string const& n, NodeType::e t)
            : Node(sched, pl, n, t),
              m_buffer(){
        }

        void init(){
            m_speed = fast;
            registerInputID(Image_In_Name);

            registerParamID<float>("rejection threshold", 0.5, "0 = never reject anything, 1.0 = reject everything");

            registerOutputID<image_ptr_t>("Stitch Buffer");
            registerOutputID<NodeParamValue>("Last Stitch Location");    // centre of first image == 0,0
            registerOutputID<NodeParamValue>("Last Stitch Orientation"); // up in first image = 0, (-180,180]
            registerOutputID<NodeParamValue>("Last Stitch Scale");       // scale of first image = 1
            registerOutputID<NodeParamValue>("Last Stitch Goodness");    // 
        }

        virtual ~StitchNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t& inputs){
            out_map_t r;
            bool first_run = false;

            if(!m_buffer){
                // first run, take a copy of the input
                first_run = true;
                m_buffer = boost::make_shared<Image>(*inputs[Image_In_Name]);
                r["Last Stitch Location"] = _mkLocation(0,0);
                r["Last Stitch Orientation"] = 0.0f;
                r["Last Stitch Scale"] = 1.0f;
                r["Last Stitch Goodness"] = 1.0f;
            }

            // even if this is the first run, proceed to stitch against the
            // buffer: as a sanity check the first image should match itself
            // perfectly!
            
            // ..... TODO
            
            r["Stitch Buffer"] = m_buffer;
            return r;
        }

    private:
        NodeParamValue _mkLocation(float x, float y){
            std::vector<float> r(2);
            r[0] = x;
            r[1] = y;
            return NodeParamValue(r);
        }

        cv::Point2i m_buffer_origin;
        image_ptr_t m_buffer;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __STITCH_NODE_H__


