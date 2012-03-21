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

#ifndef __CAUV_UTILITY_EXEC_H__
#define __CAUV_UTILITY_EXEC_H__

#include <string>
#include <cstdio>

#include "debug/cauv_debug.h"

namespace cauv{

inline static std::string exec(const char* cmd) {
    FILE* pipe = popen(cmd, "r");
    if(!pipe){
        error() << "failed to popen pipe";
        return "";
    }
    char buffer[4096];
    std::string result = "";
    while(!std::feof(pipe))
        if(std::fgets(buffer, 4096, pipe))
            result += buffer;
    pclose(pipe);
    return result;
}

} // namespace cauv

#endif // ndef __CAUV_UTILITY_EXEC_H__
