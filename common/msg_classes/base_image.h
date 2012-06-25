#ifndef __CAUV_BASEIMAGE_H__
#define __CAUV_BASEIMAGE_H__

#include <utility/serialisation-types.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/UID.h>

namespace cauv {

class BaseImage {
    friend void serialise(svec_ptr, BaseImage const&);
    friend int32_t deserialise(const_svec_ptr, uint32_t i, BaseImage&);
    public:
    BaseImage(void);
    BaseImage(svec_t const &bytes);
    BaseImage(svec_t const &bytes, TimeStamp const &ts);
    BaseImage(svec_t const &bytes, TimeStamp const &ts, UID const &id);
    BaseImage(BaseImage const &other);
    virtual ~BaseImage();
        
    TimeStamp ts() const;
    void ts(TimeStamp const &ts);

    // UID is unique to the instance in which a sensor records data, and
    // can be used to associate derived data with other data deriving from
    // the same sensor reading.
    // If data derives from multiple sensors or multiple sensor readings or
    // multiple sensors, then it should probably get a new UID at that
    // point.
    UID id() const;
    void id(UID const &id);

    //Useful for specific subclasses
    virtual svec_t &private_bytes(void) const;
    virtual void private_bytes(svec_t&);

    virtual svec_t &bytes(void) const;
    virtual void bytes(svec_t&);

    virtual uint32_t serializeQuality(void) const;
    virtual void serializeQuality(uint32_t);

    virtual uint32_t channels(void) const;
    virtual void channels(uint32_t);

    protected:
    std::string m_compress_fmt;
    TimeStamp m_ts;
    UID m_uid;
    uint32_t m_quality;
    mutable uint32_t m_channels;
    mutable svec_t m_bytes;
    mutable svec_t m_private_bytes;
};

/* image serialisation */
void serialise(svec_ptr p, BaseImage const& v);
int32_t deserialise(const_svec_ptr p, uint32_t i, BaseImage& v);
std::string chil(BaseImage const&);

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, BaseImage const& image){
    os << "BaseImage {";
    os << " ts() = " << image.ts() << ", ";
    os << " id() = " << image.id();
    os << "}";
    return os;
}

} //namespace cauv

#endif
