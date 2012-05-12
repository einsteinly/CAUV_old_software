/* Copyright 2012 Cambridge Hydronautics Ltd.
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
 
#ifndef __CAUV_COMMON_MSG_CLASS_NORTHEASTDEPTH_H__
#define __CAUV_COMMON_MSG_CLASS_NORTHEASTDEPTH_H__

#include <generated/types/floatXYZ.h>

namespace cauv{

class NorthEastDepth: public floatXYZ{
    public:
        NorthEastDepth()
            : floatXYZ(){
        }

        NorthEastDepth(floatXYZ const& a)
            : floatXYZ(a){
        }

        NorthEastDepth(float const& metres_north, float const& metres_east, float const& depth)
            : floatXYZ(metres_east, metres_north, depth){
        }

        float const& north() const{
            return y;
        }

        float const& east() const{
            return x;
        }

        float const& depth() const{
            return z;
        }

};

} // namespace cauv

#endif // ndef __CAUV_COMMON_MSG_CLASS_NORTHEASTDEPTH_H__

