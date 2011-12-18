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

// proof-of-concept:
//  BoundedFloat is declared as an externally defined type in messages.msg (ie,
//  "I promise that this type will exist, and that it will support the same
//  interface that serialisable classes do") - but in order to define the
//  tedious serialisation interface it derives from a serialisable type that
//  also defines all the state.
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

// TODO: implementations of these are only necessary because they are declared
// in generated/serialisation.h - there should be a way to suppress the
// generation of declarations in serialisation.h so that the base versions are
// used directly.
inline void serialise(svec_ptr s, BoundedFloat const& a){
    serialise(s, dynamic_cast<BoundedFloatBase const&>(a));
}
inline int32_t deserialise(const_svec_ptr s, uint32_t i, BoundedFloat& a){
    return deserialise(s, i, dynamic_cast<BoundedFloatBase&>(a));
}
inline std::string chil(BoundedFloat const& a){
    return chil(dynamic_cast<BoundedFloatBase const&>(a));
}

template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, BoundedFloat const& s)
{
    os << "BoundedFloat {";
    os << " value = " << s.value<< ",";
    os << " min = " << s.min<< ",";
    os << " max = " << s.max<< ",";
    os << " type = " << s.type;
    os << " }";
    return os;
}

} // namespace cauv

#ifdef QT_VERSION
#include <QMetaType>
Q_DECLARE_METATYPE(cauv::BoundedFloat)
#endif

#endif // ndef __CAUV_COMMON_BOUNDED_FLOAT_H__

