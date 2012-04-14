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

#include "uid.h"
#include "exec.h"

#include <sstream>

cauv::UID::UID() { }
cauv::UID::UID(int32_t const& host, int32_t const& sensor, int32_t const& seq1, int32_t const& seq2) : 
    host(host), sensor(sensor), seq1(seq1), seq2(seq2) {}

bool cauv::UID::operator==(cauv::UID const& other) const
{
    return
        host   == other.host   &&
        sensor == other.sensor &&
        seq1   == other.seq1   &&
        seq2   == other.seq2;
}

bool cauv::UID::operator<(cauv::UID const& other) const
{
    if(host < other.host)
        return true;
    else if(!(host == other.host))
        return false;
    if(sensor < other.sensor)
        return true;
    else if(!(sensor == other.sensor))
        return false;
    if(seq1 < other.seq1)
        return true;
    else if(!(seq1 == other.seq1))
        return false;
    if(seq2 < other.seq2)
        return true;
    else if(!(seq2 == other.seq2))
        return false;
    return false;
}

static uint32_t getHostID(){
    // Don't look: this gets the last 32 bits of each of the mac addresses on
    // the system, as strings...
    // (if you can do this faster and neater without the system() in a *nix
    //  compatible way, go for it...)
    std::string ids = cauv::exec(
        "ifconfig | grep ..:..:..:.. | sed \"s/.*\\(..\\):\\(..\\):\\(..\\):\\(..\\).*/\\1\\2\\3\\4/\""
    );
    // that's right, move along, you didn't see anything
    uint32_t r;
    std::stringstream s;
    s << std::hex << ids;
    s >> r;
    return r;
}

cauv::UID cauv::mkUID(uint32_t sensor, uint64_t sequence){
    static uint32_t cached_host_id = getHostID();
    return cauv::UID(cached_host_id, sensor, uint32_t(sequence>>32), uint32_t(sequence));
}
