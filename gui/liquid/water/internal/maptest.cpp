/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include "persistentMap.h"

using namespace liquid::water::internal;

template<typename key_T, typename val_T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, QMap<key_T, val_T> const& m){
    os << "map[" << m.size() << "] {";
    for(typename QMap<key_T, val_T>::const_iterator i = m.begin(); i != m.end();){
        os << i.key() << " : " << i.value();
        if(++i != m.end())
            os << ", ";
    }
    os << "}";
    return os;
}

namespace std{
template<typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, Map::MinAvgMax const& m){
    return os << "(" << m.min << ", " << m.average << ", " << m.max << ")";
}
} // namespace std

void test(){
    Map m("testmap");
    
    // this will fail if we already inserted the same values on a previous test
    // run
    try{
        for(int i = 0; i < 1000; i++){
            m.insert(0.1*i, 0.0001*i*i);
        }
    }catch(std::exception& e){
        error() << e.what();
    }

    info() << m.valueClosestTo(10.01);
    info() << m.valueClosestTo(9.99);
    info() << m.valueClosestTo(9.95);
    info() << m.valueClosestTo(10.05);

    info() << m.valuesInRange(0.0, 10.0);
    info() << m.valuesInRange(2.0, 2.1);
    info() << m.valuesInRange(2.1, 2.2);
    info() << m.valuesInRange(7.0, 12.0);
    
    Map::MinAvgMax o;
    info() << m.minAvgMaxOfRange(0,100, o) << o;
    info() << m.minAvgMaxOfRange(0,10, o) << o;
    info() << m.minAvgMaxOfRange(10,20, o) << o;

    info() << m.valuesCloseTo(5, 1);
    info() << m.valuesCloseTo(5, 2);
    info() << m.valuesCloseTo(5, 3);
    info() << m.valuesCloseTo(5, 10);
}

int main(){
    try{
        test();
    }catch(std::exception& e){
        error() << e.what();
    }
}

