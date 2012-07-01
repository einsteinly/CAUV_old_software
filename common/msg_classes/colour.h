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

#include <boost/algorithm/string.hpp>

#include <utility/serialisation-types.h>
#include <generated/types/ColourBase.h>
#include <generated/types/ColourType.h>

namespace cauv {

class Colour : public ColourBase {
    public:
        Colour() : ColourBase() {}
        Colour(ColourType::e type, boost::array<float,4> values) : ColourBase(type,values) {}

        float r() const;
        float g() const;
        float b() const;
        float a() const;
        float grey() const;

        static Colour fromRGB(float r, float g, float b);
        static Colour fromARGB(float a, float r, float g, float b);
        static Colour fromBGR(float b, float g, float r);
        static Colour fromBGRA(float b, float g, float r, float a);
        static Colour fromGrey(float grey);
};

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Colour const& c){
    os << "Colour ";
    switch (c.type) {
        case ColourType::RGB:
            return os << "RGB {" << c.r() << "," << c.g() << "," << c.b() << "}";
        case ColourType::ARGB:
            return os << "ARGB {" << c.a() << "," << c.r() << "," << c.g() << "," << c.b() << "}";
        case ColourType::BGR:
            return os << "BGR {" << c.b() << "," << c.g() << "," << c.r() << "}";
        case ColourType::BGRA:
            return os << "BGRA {" << c.b() << "," << c.g() << "," << c.r() << "," << c.a() << "}";
        case ColourType::Grey:
            return os << "Grey {" << c.grey() << "}";
        default: {
            os <<"Unknown type {";
            for (boost::array<float,4>::const_iterator it = c.values.begin(); it != c.values.end(); ++it) {
                if (it == c.values.begin()) {
                    os << ",";
                }
                os << *it;
            }
            return os;
        }
    }
}

template<typename charT, typename traits>
std::basic_istream<charT, traits>& operator>>(
    std::basic_istream<charT, traits>& is, Colour& c){
    std::string s;
    is >> s;
    if (boost::iequals(s, "Colour"))
        is >> s;
    is.ignore(1,'{');
    if (boost::iequals(s, "rgb")) {
        float r,g,b;
        is >> r;
        is.ignore(1,',');
        is >> g;
        is.ignore(1,',');
        is >> b;
        c = Colour::fromRGB(r,g,b);
    }
    else if (boost::iequals(s, "argb")) {
        float a,r,g,b;
        is >> a;
        is.ignore(1,',');
        is >> r;
        is.ignore(1,',');
        is >> g;
        is.ignore(1,',');
        is >> b;
        c = Colour::fromARGB(r,g,b,a);
    }
    else if (boost::iequals(s, "bgr")) {
        float b,g,r;
        is >> b;
        is.ignore(1,',');
        is >> g;
        is.ignore(1,',');
        is >> r;
        c = Colour::fromBGR(b,g,r);
    }
    else if (boost::iequals(s, "bgra")) {
        float b,g,r,a;
        is >> b;
        is.ignore(1,',');
        is >> g;
        is.ignore(1,',');
        is >> r;
        is.ignore(1,',');
        is >> a;
        c = Colour::fromBGRA(b,g,r,a);
    }
    else if (boost::iequals(s, "grey")) {
        float grey;
        is >> grey;
        c = Colour::fromGrey(grey);
    }
    else {
        is.setstate(std::ios_base::failbit);
    }
    is.ignore(1,'}');
    return is;
}

} //namespace cauv

#endif

