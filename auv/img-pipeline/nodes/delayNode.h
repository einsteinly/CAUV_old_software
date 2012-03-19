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

#ifndef __DELAY_NODE_H__
#define __DELAY_NODE_H__

#include "../node.h"

#include <queue>

namespace cauv{
namespace imgproc{

class DelayNode: public Node{
        const static std::string Delay_Param_Name;
    public:
        DelayNode(ConstructArgs const& args)
            : Node(args), m_queue_image_size(0,0), m_queue(){
        }

        void init(){
            m_speed = fast;

            // input:
            registerInputID(Image_In_Name);

            // parameters
            registerParamID<int>(Delay_Param_Name, 1,
                "Number of frames to delay output by: if fewer than required "
                "frames have yet been received the output is delayed by as "
                "much as possible.");

            // outputs:
            registerOutputID(Image_Out_Name);
        }

        virtual void paramChanged(input_id const& p){
            if(p == Delay_Param_Name && param<int>(Delay_Param_Name) < 0){
                warning() << "Can't set delay less than 0: If I knew the"
                          << "future I'd ";
                setParam(Delay_Param_Name, 0);
            }
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){
            
            int delay_by = param<int>("delay (frames)");

            // small chance that we're executing after an invalid parameter has
            // been set, and before it's been corrected
            if(delay_by < 0)
                delay_by = 0;
            
            image_ptr_t in = inputs[Image_In_Name];
            
            // make sure output size is (almost) always the same as input size
            // there is a race condition here, but it doesn't have any
            // disastrous effects (simultaneous modification of in->mat())
            try{
                if(in->mat().size() != m_queue_image_size){
                    m_queue = std::queue<image_ptr_t>();
                    m_queue_image_size = in->mat().size();
                }
            }catch(boost::bad_get& e){
                // oops, can't get size... hope for the best!
                // TODO: compare metadata of other sorts of image using a
                // visitor
                ;
            }

            while(m_queue.size() > unsigned(delay_by))
                m_queue.pop();
            
            m_queue.push(in);

            r[Image_Out_Name] = m_queue.front();

        }

    private:
        cv::Size m_queue_image_size;
        std::queue<image_ptr_t> m_queue;

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __DELAY_NODE_H__



