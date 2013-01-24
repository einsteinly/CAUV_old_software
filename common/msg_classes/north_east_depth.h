/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

