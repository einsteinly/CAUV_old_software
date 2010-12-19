#ifndef __CAUV_SERIALISATION_H__
#define __CAUV_SERIALISATION_H__

#include <vector>
#include <map>
#include <utility>
#include <string>

#include <boost/cstdint.hpp>

#include "serialisation-types.h"

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
} // namespace cauv::impl

/* declare overloads for supported types
 */
inline void serialise(svec_ptr p, int8_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i , int8_t& v){
    return impl::unCopyBytes(p, i, v);
}

inline void serialise(svec_ptr p, int16_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, int16_t& v){
    return impl::unCopyBytes(p, i, v);
}

inline void serialise(svec_ptr p, int32_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, int32_t& v){
    return impl::unCopyBytes(p, i, v);
}

inline void serialise(svec_ptr p, uint8_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint8_t& v){
    return impl::unCopyBytes(p, i, v);
}

inline void serialise(svec_ptr p, uint16_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint16_t& v){
    return impl::unCopyBytes(p, i, v);
}

inline void serialise(svec_ptr p, uint32_t const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, uint32_t& v){
    return impl::unCopyBytes(p, i, v);
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

inline void serialise(svec_ptr p, double const& v){
    impl::copyBytes(p, v);
}
inline int32_t deserialise(const_svec_ptr p, uint32_t i, double& v){
    return impl::unCopyBytes(p, i, v);
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

/* Define overloads for supported template types:
 */
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


} // namespace cauv

#endif // ndef __CAUV_SERIALISATION_H__

