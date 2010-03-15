#ifndef VECTOR_STREAMOPS_H
#define VECTOR_STREAMOPS_H

#include <ostream>
#include <vector>

template<typename T, typename char_T, typename traits>
std::basic_ostream<char_T, traits>& operator<<(
    std::basic_ostream<char_T, traits>& os, std::vector<T> const& a){
    os << "{vec s=" << a.size();
    for(typename std::vector<T>::const_iterator i = a.begin(); i != a.end(); i++){
        os << " " << *i;
    }
    os << "}";
    return os;
}

#endif // VECTOR_STREAMOPS_H

