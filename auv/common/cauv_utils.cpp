#include "cauv_utils.h"

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
