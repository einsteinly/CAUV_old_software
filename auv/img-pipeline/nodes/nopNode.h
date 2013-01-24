/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __NOP_NODE_H__
#define __NOP_NODE_H__

#include "../node.h"

namespace cauv{
namespace imgproc{

class NopNode: public Node{
    public:
        NopNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;

            // input:
            registerInputID(Image_In_Name, Const);

            // outputs:
            registerOutputID(Image_Out_Name);
        }

    protected:
        void doWork(in_image_map_t& inputs, out_map_t& r){

            r[Image_Out_Name] = inputs[Image_In_Name];

        }

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __NOP_NODE_H__


