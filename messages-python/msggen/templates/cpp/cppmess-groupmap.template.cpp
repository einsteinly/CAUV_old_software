\#include "groupmap.h"
/***  This is a generated file, do not edit ***/

namespace cauv {

std::vector<uint32_t> get_ids_for_group(const std::string group) {
    std::vector<uint32_t> output_vect;
    if(group == "") {
    }#for $g in $groups
    else if (group == "${g.name}") {
    #for $m in $g.messages
        output_vect.push_back($m.id);
    #end for
    }
    #end for
    return output_vect;
}

}
