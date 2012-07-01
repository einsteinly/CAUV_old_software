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

#ifndef __CAUV_COLOUR_H__
#define __CAUV_COLOUR_H__

#include <utility/serialisation-types.h>
#include <generated/types/ColourBase.h>
#include <generated/types/ColourType.h>

namespace cauv {

class Colour : public ColourBase {
    public:
        float r() const;
        float g() const;
        float b() const;
        float a() const;
        float grey() const;
};

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Colour const& c){
    os << "Colour {";
    if (c.type == ColourType::RGB) {
        os << " r = " << c.r() << ", ";
        os << " g = " << c.g() << ", ";
        os << " b = " << c.b();
    }
    else {
        os << " grey = " << c.grey();
    }
    os << "}";
    return os;
}

} //namespace cauv

#endif

