/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
            registerParamID<T>("A (triggers exec)", T(), "value that must be new for execution", Must_Be_New);
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

