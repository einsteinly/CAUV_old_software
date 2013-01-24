/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __VALUE_INPUT_NODE_H__
#define __VALUE_INPUT_NODE_H__

#include "../../node.h"

namespace cauv{
namespace imgproc{

template<typename Value_T>
class ValueInputNode: public Node{
    public:
        ValueInputNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            registerParamID<Value_T>("value", Value_T(), "input value");
            registerOutputID("value", Value_T());
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            const Value_T value = param<Value_T>("value");
            r["value"] = ParamValue(value);
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __VALUE_INPUT_NODE_H__
