/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "colour.h"

#include <limits>
#include <utility/math.h>

float cauv::Colour::r() const {
    switch (type) {
        // Always store BGRA
        case ColourType::RGB:
        case ColourType::ARGB:
        case ColourType::BGR:
        case ColourType::BGRA:
            return values[2];
        case ColourType::Grey:
            return values[0];
        default:
            return std::numeric_limits<float>::quiet_NaN();
    }
}
float cauv::Colour::g() const {
    switch (type) {
        // Always store BGRA
        case ColourType::RGB:
        case ColourType::ARGB:
        case ColourType::BGR:
        case ColourType::BGRA:
            return values[1];
        case ColourType::Grey:
            return values[0];
        default:
            return std::numeric_limits<float>::quiet_NaN();
    }
}
float cauv::Colour::b() const {
    switch (type) {
        // Always store BGRA
        case ColourType::RGB:
        case ColourType::ARGB:
        case ColourType::BGR:
        case ColourType::BGRA:
            return values[0];
        case ColourType::Grey:
            return values[0];
        default:
            return std::numeric_limits<float>::quiet_NaN();
    }
}
float cauv::Colour::a() const {
    switch (type) {
        // Always store BGRA
        case ColourType::BGRA:
        case ColourType::ARGB:
            return values[3];
        case ColourType::RGB:
        case ColourType::BGR:
        case ColourType::Grey:
            return 1;
        default:
            return std::numeric_limits<float>::quiet_NaN();
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
            return 0.2989 * r() + 0.5870 * g() + 0.1140 * b();
        default:
            return std::numeric_limits<float>::quiet_NaN();
    }
}

cauv::Colour cauv::Colour::fromRGB(float r, float g, float b) {
    boost::array<float,4> values = {{b,g,r,1}};
    return cauv::Colour(ColourType::RGB, values);
}
cauv::Colour cauv::Colour::fromARGB(float a, float r, float g, float b) {
    boost::array<float,4> values = {{b,g,r,a}};
    return cauv::Colour(ColourType::ARGB, values);
}
cauv::Colour cauv::Colour::fromBGR(float b, float g, float r) {
    boost::array<float,4> values = {{b,g,r,1}};
    return cauv::Colour(ColourType::BGR, values);
}
cauv::Colour cauv::Colour::fromBGRA(float b, float g, float r, float a) {
    boost::array<float,4> values = {{b,g,r,a}};
    return cauv::Colour(ColourType::BGRA, values);
}
cauv::Colour cauv::Colour::fromGrey(float grey) {
    boost::array<float,4> values = {{grey,grey,grey,1}};
    return cauv::Colour(ColourType::Grey, values);
}
        
cauv::Colour& cauv::Colour::operator+= (const cauv::Colour& that) {
    boost::array<float,4>::iterator thisit, thisend;
    boost::array<float,4>::const_iterator thatit;
    
    for (thisit = this->values.begin(), thisend = this->values.end(), thatit = that.values.begin();
         thisit != thisend;
         ++thisit, ++thatit) {
        *thisit += *thatit;
    }
    return *this;
}
cauv::Colour& cauv::Colour::operator-= (const cauv::Colour& that) {
    boost::array<float,4>::iterator thisit, thisend;
    boost::array<float,4>::const_iterator thatit;
    
    for (thisit = this->values.begin(), thisend = this->values.end(), thatit = that.values.begin();
         thisit != thisend;
         ++thisit, ++thatit) {
        *thisit -= *thatit;
    }
    return *this;
}
cauv::Colour& cauv::Colour::operator+= (float val) {
    boost::array<float,4>::iterator thisit, thisend;
    
    for (thisit = this->values.begin(), thisend = this->values.end();
         thisit != thisend;
         ++thisit) {
        *thisit += val;
    }
    return *this;
}
cauv::Colour& cauv::Colour::operator-= (float val) {
    boost::array<float,4>::iterator thisit, thisend;
    
    for (thisit = this->values.begin(), thisend = this->values.end();
         thisit != thisend;
         ++thisit) {
        *thisit -= val;
    }
    return *this;
}
cauv::Colour& cauv::Colour::operator*= (float val) {
    boost::array<float,4>::iterator thisit, thisend;
    
    for (thisit = this->values.begin(), thisend = this->values.end();
         thisit != thisend;
         ++thisit) {
        *thisit *= val;
    }
    return *this;
}
cauv::Colour& cauv::Colour::operator/= (float val) {
    boost::array<float,4>::iterator thisit, thisend;
    
    for (thisit = this->values.begin(), thisend = this->values.end();
         thisit != thisend;
         ++thisit) {
        *thisit /= val;
    }
    return *this;
}

float cauv::sqdiff(const Colour& c1, const Colour& c2) {
    float sqdiff = 0;
    sqdiff += sq(c1.a() - c2.a());
    
    if (c1.type != ColourType::Grey || c2.type != ColourType::Grey)
        sqdiff += sq(c1.r() - c2.r()) + sq(c1.g() - c2.g()) + sq(c1.b() - c2.b());
    else
        sqdiff += sq(c1.grey() - c2.grey());

    return sqdiff;
}
