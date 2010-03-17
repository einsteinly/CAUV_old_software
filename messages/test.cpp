#include <iostream>
#include <iomanip>
#include <vector>
#ifndef foreach
#   include <boost/foreach.hpp>
#   define foreach BOOST_FOREACH
#endif

#include "messages.h"

using namespace std;

int main()
{
    string bytes;


    DebugMessage m3;
    m3.msg("Hello");

    bytes = m3.toBytes();

    foreach(char c, bytes)
    {
        cout << hex << setw(2) << setfill('0') << (unsigned int)*reinterpret_cast<u_char*>(&c) << " ";
    }
    cout << endl;


    MotorMessage m;
    
    m.speed(30);
    m.motorId(PROP);

    bytes = m.toBytes();

    foreach(char c, bytes)
    {
        cout << hex << setw(2) << setfill('0') << (unsigned int)*reinterpret_cast<u_char*>(&c) << " ";
    }
    cout << endl;
    
    
    ImageMessage m2;
    m2.type(0xfe);
    vector<uint8_t> image_bytes;
    for (size_t i = 0; i < 50l; ++i)
    {
        image_bytes.push_back(1);
        image_bytes.push_back(2);
        image_bytes.push_back(3);
    }
    m2.image(image_bytes);

    bytes = m2.toBytes();

    foreach(char c, bytes)
    {
        cout << hex << setw(2) << setfill('0') << (unsigned int)*reinterpret_cast<u_char*>(&c) << " ";
    }
    cout << endl;

    return 0;
}
