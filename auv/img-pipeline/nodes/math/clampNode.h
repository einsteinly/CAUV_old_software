/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CLAMP_NODE_H__
#define __CLAMP_NODE_H__

#include "../../node.h"

#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/core/core_c.h"
#include "opencv2/features2d/features2d.hpp"

#include <utility/rounding.h>

namespace cauv{
namespace imgproc{

template<typename T>
class ClampNode: public Node{
    public:
        ClampNode(ConstructArgs const& args)
            : Node(args){
        }

        void init(){
            m_speed = fast;

            // input:
            registerParamID<T>("Value", T(), "changing value", Must_Be_New);
            registerParamID<T>("Min", T(), "changing value");
            registerParamID<T>("Max", T(), "changing value");

            // outputs:
            registerOutputID("Value", T());
        }

    protected:
        void doWork(in_image_map_t&, out_map_t& r){
            T min   = param<T>("Min");
            T max   = param<T>("Max");
            T value = param<T>("Value");
            r["Value"] = clamp(min, value, max);
        }
    
    private:

    // Register this node type
    DECLARE_NFR;
};

} // namespace imgproc
} // namespace cauv

#endif // ndef __CLAMP_NODE_H__



