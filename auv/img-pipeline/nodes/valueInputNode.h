#ifndef __VALUE_INPUT_NODE_H__
#define __VALUE_INPUT_NODE_H__

#include "../node.h"

#include <algorithm>

namespace cauv{
namespace imgproc{

template<typename Value_T>
class ValueInputNode: public Node{
    public:
        ValueInputNode(Scheduler& sched, ImageProcessor& pl, NodeType::e t)
            : Node(sched, pl, t), m_counter(0){
        }

        void init(){
            registerParamID<Value_T>("value", 0, "input value");
            registerOutputID<NodeParamValue>("value");
        }
    
        virtual ~ValueInputNode(){
            stop();
        }

    protected:
        out_map_t doWork(in_image_map_t&){
            out_map_t r;
            const Value_T value = param<Value_T>("value");
            r["value"] = NodeParamValue(value);
            return r;
        }

        int m_counter;
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __VALUE_INPUT_NODE_H__
