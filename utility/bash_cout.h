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

#ifndef __BASH_COUT_H__
#define __BASH_COUT_H__

#include <iomanip>
#include <ostream>

#include <utility/enum_class.h>

/* bash colours */
ENUM_CLASS(BashColour,char,
        None   = 0,
        Black  = 30,
        Red    = 31,
        Green  = 32,
        Brown  = 33,
        Blue   = 34,
        Purple = 35,
        Cyan   = 36,
        White  = 37
);
ENUM_CLASS(BashBackground,char,
        Black   = 40,
        Red     = 41,
        Green   = 42,
        Yellow  = 43,
        Blue    = 44,
        Magenta = 45,
        Cyan    = 46,
        White   = 47
);
ENUM_CLASS(BashControl,char,
        Reset  = 0
);
ENUM_CLASS(BashIntensity,char,
        Bold   = 1,
        Faint  = 1,
        Normal = 22
);


template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, BashControl const& c)
{
    os << "\033[0" << int(c) << "m";
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, BashIntensity const& c)
{
    os << "\033[0" << int(c) << "m";
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, BashColour const& c)
{
    if(c == BashColour::None)
        os << BashControl::Reset;
    else
        os << "\033[0;" << int(c) << "m";
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, BashBackground const& c)
{
    os << "\033[0;" << int(c) << "m";
    return os;
}


#endif//__BASH_COUT_H__
