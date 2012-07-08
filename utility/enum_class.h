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

#ifndef __CAUV_UTILITY_ENUMCLASS_H__
#define __CAUV_UTILITY_ENUMCLASS_H__

#define ENUM_CLASS_NAMESPACE __enums
#define ENUM_CLASS(NAME, TYPE, VALUES...) \
    namespace ENUM_CLASS_NAMESPACE {\
    struct NAME { \
        enum _e { VALUES }; \
        NAME() : val() {} \
        NAME(_e v) : val(v) {} \
        explicit NAME(TYPE v) : val(v) {} \
        operator _e() const { return _e(val); } \
        NAME& operator|=(NAME::_e const& r){ \
            val |= r; \
            return *this; \
        } \
        NAME& operator&=(NAME::_e const& r){ \
            val &= r; \
            return *this; \
        } \
        private:\
            TYPE val; \
    }; \
    static inline NAME operator|(NAME::_e const& l, NAME::_e const& r){ \
        return NAME(l | r); \
    } \
    static inline NAME operator&(NAME::_e const& l, NAME::_e const& r){ \
        return NAME(l | r); \
    } \
    } \
    using ENUM_CLASS_NAMESPACE::NAME

#define ENUM_CLASS_FWD(NAME) namespace ENUM_CLASS_NAMESPACE { struct NAME; } using ENUM_CLASS_NAMESPACE::NAME

#endif
