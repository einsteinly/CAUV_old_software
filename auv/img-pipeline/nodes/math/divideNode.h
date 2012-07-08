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

#ifndef __MATH_DIVIDE_NODE_H__
#define __MATH_DIVIDE_NODE_H__

#include <opencv2/core/core.hpp>

#include "../../node.h"

namespace cauv{
namespace imgproc{

template<typename T>
class MathDivideNode: public Node{
    public:
        MathDivideNode(ConstructArgs const& args) : Node(args){ }

        void init(){
            m_speed = fast;
            
            registerOutputID("A/B", float());

            // parameters:
            registerParamID<T>("A (triggers exec)", T(), "value that must be new for execution", InputSchedType::Must_Be_New);
            registerParamID<T>("B", T());
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            T a = param<T>("A (triggers exec)");
            T b = param<T>("B");
            if(b != T(0))
                r["A/B"] = ParamValue(float(a)/float(b));
            else
                r["A/B"] = ParamValue(float(0));
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MATH_DIVIDE_NODE_H__

