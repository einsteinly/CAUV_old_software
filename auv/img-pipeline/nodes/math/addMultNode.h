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

#ifndef __MATH_ADDMULT_NODE_H__
#define __MATH_ADDMULT_NODE_H__

#include <map>
#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

#include <common/cauv_utils.h>

#include "../../node.h"


namespace cauv{
namespace imgproc{

template<typename T>
class MathAddMultNode: public Node{
    public:
        MathAddMultNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;
            
            registerOutputID("A*Afac + B*Bfac", T());

            // parameters:
            registerParamID<T>("A (triggers exec)", T(), "value that must be new for execution", Must_Be_New);
            registerParamID<float>("Afac", 1);
            registerParamID<T>("B", T());
            registerParamID<float>("Bfac", 1);
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            T a = param<T>("A (triggers exec)");
            T b = param<T>("B");
            float a_fac = param<float>("Afac");
            float b_fac = param<float>("Bfac");
            r["A*Afac + B*Bfac"] = ParamValue(T(a*a_fac + b*b_fac));
        }
    
    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __MATH_ADDMULT_NODE_H__

