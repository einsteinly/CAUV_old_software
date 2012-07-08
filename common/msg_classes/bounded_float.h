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
        BoundedFloat(float value, float min, float max, BoundedFloatType const& t)
            : BoundedFloatBase(value, min, max, t){
        }
        operator float() const{
            return this->value;
        }
};

} // namespace cauv

#endif // ndef __CAUV_COMMON_BOUNDED_FLOAT_H__

