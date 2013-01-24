/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_COMMON_BOUNDED_FLOAT_H__
#define __CAUV_COMMON_BOUNDED_FLOAT_H__

#include <generated/types/BoundedFloatBase.h>
#include <generated/types/BoundedFloatType.h>

namespace cauv{

// example:
//  BoundedFloat is declared as an externally defined type in messages.msg 
//  via BoundedFloatType subclass BoundedFloat : <common/bounded_float.h>
//  it then must only use the state as defined in BoundedFloatBase in order to
//  be serialised properly
class BoundedFloat: public BoundedFloatBase{
    public:
        BoundedFloat()
            : BoundedFloatBase(){
        }
        BoundedFloat(float value, float min, float max, BoundedFloatType::e const& t)
            : BoundedFloatBase(value, min, max, t){
        }
        operator float() const{
            return this->value;
        }
};

} // namespace cauv

#endif // ndef __CAUV_COMMON_BOUNDED_FLOAT_H__

