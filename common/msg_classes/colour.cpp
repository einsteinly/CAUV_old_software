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

#include "colour.h"

#include <limits>

float cauv::Colour::r() const {
    switch (type) {
        case ColourType::RGB:
            return values[0];
        case ColourType::BGR:
        case ColourType::BGRA:
            return values[2];
        case ColourType::ARGB:
            return values[1];
        case ColourType::Grey:
        default:
            return values[0];
    }    
}
float cauv::Colour::g() const {
    switch (type) {
        case ColourType::RGB:
        case ColourType::BGR:
        case ColourType::BGRA:
        case ColourType::ARGB:
            return values[1];
        case ColourType::Grey:
        default:
            return values[0];
    }    
}
float cauv::Colour::b() const {
    switch (type) {
        case ColourType::RGB:
            return values[2];
        case ColourType::BGR:
        case ColourType::BGRA:
            return values[0];
        case ColourType::ARGB:
            return values[3];
        case ColourType::Grey:
        default:
            return values[0];
    }    
}
float cauv::Colour::a() const {
    switch (type) {
        case ColourType::BGRA:
            return values[3];
        case ColourType::ARGB:
        case ColourType::RGB:
        case ColourType::BGR:
        case ColourType::Grey:
        default:
            return 1;
    }    
}
float cauv::Colour::grey() const {
    switch (type) {
        case ColourType::Grey:
            return values[0];
        case ColourType::BGRA:
        case ColourType::ARGB:
        case ColourType::RGB:
        case ColourType::BGR:
        default:
            return 0.2989 * r() + 0.5870 * g() + 0.1140 * b();
    }    
}

cauv::Colour cauv::Colour::fromRGB(float r, float g, float b) {
    boost::array<float,4> values = {{r,g,b,0}};    
    return cauv::Colour(ColourType::RGB, values);
}
cauv::Colour cauv::Colour::fromARGB(float a, float r, float g, float b) {
    boost::array<float,4> values = {{a,r,g,b}};    
    return cauv::Colour(ColourType::RGB, values);
}
cauv::Colour cauv::Colour::fromBGR(float b, float g, float r) {
    boost::array<float,4> values = {{b,g,r,0}};    
    return cauv::Colour(ColourType::RGB, values);
}
cauv::Colour cauv::Colour::fromBGRA(float b, float g, float r, float a) {
    boost::array<float,4> values = {{b,g,r,a}};    
    return cauv::Colour(ColourType::RGB, values);
}
cauv::Colour cauv::Colour::fromGrey(float grey) {
    boost::array<float,4> values = {{grey,0,0,0}};    
    return cauv::Colour(ColourType::RGB, values);
}
