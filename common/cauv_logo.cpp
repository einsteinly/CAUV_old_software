/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <iostream>
#include <sstream>

#include <utility/bash_cout.h>

#include "cauv_logo.h"

namespace cauv {

static void print_logo(
    const char* colour_start,
    const char* colour_end,
    const char* shape_start,
    const char* shape_end,
    const std::string& module_name
){
    const char* colour_cur = colour_start;
    const char* shape_cur = shape_start;
    std::stringstream ss;
    ss << std::endl;
    while (colour_cur != colour_end && shape_cur < shape_end)
    {
        if (*colour_cur == '\n')
            ss << BashControl::Reset << std::endl;
        else if (*shape_cur == *(shape_cur-1) && *colour_cur == *(colour_cur-1))
            ss << *shape_cur;
        else
        {
            switch(*colour_cur)
            {
                case 'K':
                    ss << BashBackground::Black;
                    break;
                case 'R':
                    ss << BashBackground::Red;
                    break;
                case 'G':
                    ss << BashBackground::Green;
                    break;
                case 'Y':
                    ss << BashBackground::Yellow;
                    break;
                case 'B':
                    ss << BashBackground::Blue;
                    break;
                case 'b':
                    ss << BashControl::Reset << BashColour::Blue;
                    break;
                case 'M':
                    ss << BashBackground::Magenta;
                    break;
                case 'C':
                    ss << BashBackground::Cyan;
                    break;
                case 'W':
                    ss << BashBackground::White;
                    break;
                case 'w':
                    ss << BashControl::Reset << BashColour::White;
                    break;
                case ' ':
                    ss << BashControl::Reset;
                    break;
                case 'c':
                {
                    ss << BashColour::Blue << BashIntensity::Bold;
                    int len = 0;
                    while (colour_cur != colour_end && *colour_cur == 'c')
                    {
                        len++;
                        colour_cur++;
                        shape_cur++;
                    }
                    if (len >= (int)module_name.size())
                    {
                        int start = (len - module_name.size())/2;
                        std::string pad(start, ' ');
                        ss << pad << module_name << pad;
                    }
                    else
                    {
                        ss << module_name.substr(0, len);
                    }
                }
                    break;
                default:
                    break;
            }
            ss << *shape_cur;
        }
        colour_cur++;
        shape_cur++;
    }
    std::cout << ss.str();
}


void print_module_header(const std::string& module_name)
{
    static char cauv_logo_colours[] = {
        #include <generated/cauv_logo_colours.h>
    };
    static char cauv_logo_shape[] = {
        #include <generated/cauv_logo_shape.h>
    };

    print_logo(cauv_logo_colours,
               cauv_logo_colours + sizeof(cauv_logo_colours),
               cauv_logo_shape,
               cauv_logo_shape + sizeof(cauv_logo_shape),
               module_name);
}

}
