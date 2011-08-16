#ifndef __CAUV_SERIALISATION_H__
#define __CAUV_SERIALISATION_H__

#include <vector>
#include <map>
#include <utility>
#include <string>
#include <iomanip>

#include <boost/cstdint.hpp>

#include "serialisation-types.h"
#include "string.h"

namespace cauv{

/* add the bytes representing T to the back of the vector:
 *
 * void serialise(svec_ptr, T const&);
 *
  * deserialise from specified position, don't remove any bytes
 *
 * int32_t deserialise(const_svec_ptr, uint32_t, T&);
 */

namespace impl{
/* helper function: serialise by copying bytes */
template<typename T>
inline static void copyBytes(svec_ptr p, T& v){
    p->insert(
        p->end(),
        reinterpret_cast<byte const*>(&v),
        reinterpret_cast<byte const*>(&v) + sizeof(T)
    );
}
template<typename T>
inline static int32_t unCopyBytes(const_svec_ptr p, uint32_t i, T& v){
    v = reinterpret_cast<T const&>((*p)[i]);
    return sizeof(T);
}

const static char* B16_LUT[] = {
    "00","01","02","03","04","05","06","07","08","09","0a","0b","0c","0d","0e","0f",
    "10","11","12","13","14","15","16","17","18","19","1a","1b","1c","1d","1e","1f",
    "20","21","22","23","24","25","26","27","28","29","2a","2b","2c","2d","2e","2f",
    "30","31","32","33","34","35","36","37","38","39","3a","3b","3c","3d","3e","3f",
    "40","41","42","43","44","45","46","47","48","49","4a","4b","4c","4d","4e","4f",
    "50","51","52","53","54","55","56","57","58","59","5a","5b","5c","5d","5e","5f",
    "60","61","62","63","64","65","66","67","68","69","6a","6b","6c","6d","6e","6f",
    "70","71","72","73","74","75","76","77","78","79","7a","7b","7c","7d","7e","7f",
    "80","81","82","83","84","85","86","87","88","89","8a","8b","8c","8d","8e","8f",
    "90","91","92","93","94","95","96","97","98","99","9a","9b","9c","9d","9e","9f",
    "a0","a1","a2","a3","a4","a5","a6","a7","a8","a9","aa","ab","ac","ad","ae","af",
    "b0","b1","b2","b3","b4","b5","b6","b7","b8","b9","ba","bb","bc","bd","be","bf",
    "c0","c1","c2","c3","c4","c5","c6","c7","c8","c9","ca","cb","cc","cd","ce","cf",
    "d0","d1","d2","d3","d4","d5","d6","d7","d8","d9","da","db","dc","dd","de","df",
    "e0","e1","e2","e3","e4","e5","e6","e7","e8","e9","ea","eb","ec","ed","ee","ef",
    "f0","f1","f2","f3","f4","f5","f6","f7","f8","f9","fa","fb","fc","fd","fe","ff"
};
} // namespace cauv::impl

/* declare overloads for supported types
 */
inline void serialise(svec_ptr p, int8_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i , int8_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(int8_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, int16_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, int16_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(int16_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, int32_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, int32_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(int32_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, uint8_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint8_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(uint8_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, uint16_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint16_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(uint16_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, uint32_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint32_t& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(uint32_t const& v){
    return mkStr() << v;
}

inline void serialise(svec_ptr p, char const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, char& v){
    return impl::unCopyBytes(p, i, v);
} 

inline void serialise(svec_ptr p, float const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, float& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(float const& v){
    return mkStr() << std::setprecision(8) << v;
}

inline void serialise(svec_ptr p, double const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, double& v){
    return impl::unCopyBytes(p, i, v);
}
inline std::string chil(double const& v){
    return mkStr() << std::setprecision(16) << v;
}

inline void serialise(svec_ptr p, bool const& v){
    serialise(p, int32_t(v));
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, bool& v){
    int32_t t = 0;
    uint32_t r = deserialise(p, i, t);
    v = t;
    return r;
}
inline std::string chil(bool const& v){
    return mkStr() << std::noboolalpha << v;
}

inline void serialise(svec_ptr p, std::string const& v){
    serialise(p, uint32_t(v.size()));
    p->insert(p->end(), (byte const*)v.data(), (byte const*)v.data()+v.size());
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::string& v){
    uint32_t n = 0;
    i += deserialise(p, i, n);
    v.assign((char*)&(p->operator[](i)), n);
    return n + 4;
}
inline std::string chil(std::string const& v){
    std::string r;
    r.reserve(v.size()*2);
    std::string::const_iterator i;
    for(i = v.begin(); i != v.end(); i++)
        r += impl::B16_LUT[unsigned(*i)];
    return r;
}

/* Define overloads for supported template types:
 */
/* prototypes:
 */
template<typename T>
inline void serialise(svec_ptr p, std::vector<T> const& v);
template<typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::vector<T>& v);

template<typename S, typename T>
inline void serialise(svec_ptr p, std::pair<S,T> const& v);
template<typename S, typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::pair<S,T>& v);

template<typename S, typename T>
inline void serialise(svec_ptr p, std::map<S,T> const& v);
template<typename S, typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::map<S,T>& v);


template<typename T>
inline void serialise(svec_ptr p, std::vector<T> const& v){
    assert(v.size() < 0x80000000);
    int32_t num = int32_t(v.size());
    serialise(p, num);
    
    typename std::vector<T>::const_iterator i;
    for(i = v.begin(); i != v.end(); i++)
        serialise(p, *i);
}

template<typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::vector<T>& v){
    int32_t b = i;
    int32_t num = 0;
    b += deserialise(p, b, num);
    
    v.reserve(num);
    /* NB: assuming v is already clear */
    for(int32_t j = 0; j < num; j++){
        T t;
        b += deserialise(p, b, t);
        v.push_back(t);
    }
    return b - i;
}

template<typename T>
inline std::string chil(std::vector<T> const& V){
    std::string r = "(";
    std::size_t i;
    for(i = 0; i < V.size() && i+1 != V.size(); i++)
        r += chil(V[i]) + ",";
    if(i+1 == V.size())
        r += chil(V[i]);
    return r + ")";
}

template<typename S, typename T>
inline void serialise(svec_ptr p, std::pair<S,T> const& v){
    serialise(p, v.first);
    serialise(p, v.second);
}

template<typename S, typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::pair<S,T>& v){
    uint32_t b = i;
    b += deserialise(p, b, v.first);
    b += deserialise(p, b, v.second);
    return b - i;
}
template<typename S, typename T>
inline std::string chil(std::pair<S,T> const& v){
    return mkStr() << "(" << chil(v.first) << "," << chil(v.second) << ")";
}


template<typename S, typename T>
inline void serialise(svec_ptr p, std::map<S,T> const& v){
    assert(v.size() < 0x80000000);
    int32_t num = int32_t(v.size());
    serialise(p, num);

    typename std::map<S,T>::const_iterator i;
    for(i = v.begin(); i != v.end(); i++)
        serialise(p, *i);
}

template<typename S, typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::map<S,T>& v){
    int32_t b = i;
    int32_t num = 0;
    b += deserialise(p, b, num);
    
    /* NB: assuming v is already clear */
    for(int32_t j = 0; j < num; j++){
        std::pair<S, T> t;
        b += deserialise(p, b, t);
        /* hint that this element will be placed at the end */
        v.insert(v.end(), t);
    }
    return b - i;
}

template<typename S, typename T>
inline std::string chil(std::map<S,T> const& V){
    std::string r = "("; 
    typename std::map<S,T>::const_iterator i;
    for(i = V.begin(); i != V.end();){
        r += chil(*i);
        if(++i != V.end())
            r += ",";
    }
    return r + ")";
}


} // namespace cauv

#endif // ndef __CAUV_SERIALISATION_H__

