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

#ifndef __CAUV_UTILITY_UID_H__
#define __CAUV_UTILITY_UID_H__

#include <stdint.h>

namespace cauv {

struct UID
{
    uint32_t host;
    int32_t sensor;
    int32_t seq1;
    int32_t seq2;
        
    UID();
    UID(uint32_t const& host, int32_t const& sensor, int32_t const& seq1, int32_t const& seq2);

    bool operator==(UID const& other) const;
    bool operator<(UID const& other) const;
};

UID mkUID(uint32_t sensor=0, uint64_t sequence=0);

}

#endif
