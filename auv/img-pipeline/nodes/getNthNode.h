/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __GETNTH_NODE_H__
#define __GETNTH_NODE_H__

#include <vector>

#include "../node.h"
#include "../nodeFactory.h"
#include "../pipelineTypes.h"


namespace cauv{
namespace imgproc{

template<typename T>
class GetNthNode: public Node{
    public:
        GetNthNode(ConstructArgs const& args) :
                Node(args)
        {
        }

        void init()
        {
            // fast node:
            m_speed = fast;

            // one input:

            // two outputs:
            registerOutputID("value", T());

            // parameters:
            registerParamID("list", std::vector<T>());
            registerParamID<int>("index", 0);
        }
        
    protected:

        void doWork(in_image_map_t& , out_map_t& r){
            std::vector<T> list = param<std::vector<T> >("list");
            int index = param<int>("index");

            if (index < 0 || index >= (int)list.size())
                error() << "Index" << index << "out of range (0, " << list.size() << ")";
            else
                r["value"] = list[index];
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __GETNTH_NODE_H__
