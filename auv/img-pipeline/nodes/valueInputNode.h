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

#ifndef __VALUE_INPUT_NODE_H__
#define __VALUE_INPUT_NODE_H__

#include "../node.h"

#include <algorithm>

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
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __VALUE_INPUT_NODE_H__
