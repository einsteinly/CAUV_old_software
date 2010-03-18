#include "cauv_utils.h"

#include <common/messages.h>

using namespace std;

MakeString::operator string() const {
    return stream.str();
}

uint16_t sumOnesComplement(vector<uint16_t> bytes)
{
    uint32_t sum = 0;
    foreach(uint16_t byte, bytes)
    {
        sum += byte;
    }
    while (sum >> 16)
        sum = (sum >> 16) + (sum & 0xffff);

    return ~(uint16_t)sum;
}


TimeStamp now(){
    TimeStamp r;
    boost::posix_time::ptime epoch(boost::gregorian::date(1970,1,1));
    boost::posix_time::ptime current_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff = current_time - epoch;
    r.secs = diff.total_seconds();
    r.msecs = diff.fractional_seconds();
    return r;
}
