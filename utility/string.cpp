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

#include <utility/string.h>

std::string cauv::implode( const std::string &glue, const std::set<std::string> &pieces )
{
    std::string a;
    int leng=pieces.size();
    
    std::set<std::string>::iterator i;
    for (i=pieces.begin(); i!=pieces.end(); i++){
        a += *i;
        if (--leng)
            a += glue;
    }
    return a;
}

