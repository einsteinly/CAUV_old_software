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

#define CAUV_RESTRICT __restrict__ 

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
inline static void copyBytes(svec_ptr p, T& CAUV_RESTRICT v){
    svec_ptr::element_type* const CAUV_RESTRICT restrict_p = p.get();
    restrict_p->insert(
        restrict_p->end(),
        reinterpret_cast<byte const*>(&v),
        reinterpret_cast<byte const*>(&v) + sizeof(T)
    );
}
template<typename T>
inline static int32_t unCopyBytes(const_svec_ptr p, uint32_t i, T& CAUV_RESTRICT v){
    v = reinterpret_cast<T const&>((*p)[i]);
    return sizeof(T);
}

const static char* B16_LUT[] = {
    "00","01","02","03","04","05","06","07","08","09","0A","0B","0C","0D","0E","0F",
    "10","11","12","13","14","15","16","17","18","19","1A","1B","1C","1D","1E","1F",
    "20","21","22","23","24","25","26","27","28","29","2A","2B","2C","2D","2E","2F",
    "30","31","32","33","34","35","36","37","38","39","3A","3B","3C","3D","3E","3F",
    "40","41","42","43","44","45","46","47","48","49","4A","4B","4C","4D","4E","4F",
    "50","51","52","53","54","55","56","57","58","59","5A","5B","5C","5D","5E","5F",
    "60","61","62","63","64","65","66","67","68","69","6A","6B","6C","6D","6E","6F",
    "70","71","72","73","74","75","76","77","78","79","7A","7B","7C","7D","7E","7F",
    "80","81","82","83","84","85","86","87","88","89","8A","8B","8C","8D","8E","8F",
    "90","91","92","93","94","95","96","97","98","99","9A","9B","9C","9D","9E","9F",
    "A0","A1","A2","A3","A4","A5","A6","A7","A8","A9","AA","AB","AC","AD","AE","AF",
    "B0","B1","B2","B3","B4","B5","B6","B7","B8","B9","BA","BB","BC","BD","BE","BF",
    "C0","C1","C2","C3","C4","C5","C6","C7","C8","C9","CA","CB","CC","CD","CE","CF",
    "D0","D1","D2","D3","D4","D5","D6","D7","D8","D9","DA","DB","DC","DD","DE","DF",
    "E0","E1","E2","E3","E4","E5","E6","E7","E8","E9","EA","EB","EC","ED","EE","EF",
    "F0","F1","F2","F3","F4","F5","F6","F7","F8","F9","FA","FB","FC","FD","FE","FF"
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
    return mkStr() << int32_t(v);
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
    return mkStr() << uint32_t(v);
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
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint32_t& CAUV_RESTRICT v){
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
inline void serialise(svec_ptr p, std::vector<T> const& CAUV_RESTRICT v);
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
inline void serialise(svec_ptr p, std::vector<T> const& CAUV_RESTRICT v){
    assert(v.size() < 0x80000000);
    int32_t num = int32_t(v.size());
    // 2 * p->size to preserve O(Length) amortized time with the gcc
    // reserve-means-reserve-exactly implementation...
    const std::size_t add_length = sizeof(int32_t) + (sizeof(T)) * num;
    if(p->capacity() < p->size() + add_length)
        p->reserve(2*p->size() + add_length);
    serialise(p, num);
    
    typename std::vector<T>::const_iterator i;
    for(i = v.begin(); i != v.end(); i++)
        serialise(p, *i);
}

template<typename T>
inline int32_t deserialise(const_svec_ptr p, uint32_t i, std::vector<T>& CAUV_RESTRICT v){
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
    mkStr r;
    r << "(";
    std::size_t i;
    for(i = 0; i < V.size() && i+1 != V.size(); i++)
        r << chil(V[i]) << ",";
    if(i+1 == V.size())
        r << chil(V[i]);
    return r << ")";
}

template<>
inline std::string chil(std::vector<byte> const& V){
    std::string r;
    r.reserve(V.size()*2);
    std::vector<byte>::const_iterator i;
    for(i = V.begin(); i != V.end(); i++)
        r += impl::B16_LUT[unsigned(*i)];
    return r;
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

    // 2 * p->size to preserve O(Length) amortized time with the gcc
    // reserve-means-reserve-exactly implementation...
    const std::size_t add_length = sizeof(int32_t) + num * (sizeof(S)+sizeof(T));
    if(p->capacity() < p->size() + add_length)
        p->reserve(2*p->size() + add_length);
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
    mkStr r;
    r << "(";
    typename std::map<S,T>::const_iterator i;
    for(i = V.begin(); i != V.end();){
        r << chil(*i);
        if(++i != V.end())
            r << ",";
    }
    return r << ")";
}


} // namespace cauv

#endif // ndef __CAUV_SERIALISATION_H__

