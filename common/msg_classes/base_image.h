/* Copyright 2011-2012 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Steve Ogborne   steve@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */


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
    virtual svec_t const& private_bytes() const;
    virtual void private_bytes(svec_t&);
    
    // for thread-safety encodeBytes doesn't set the internal encoded bytes
    // buffer
    virtual svec_t const& encodedBytes() const;
    // derived types must override this:
    virtual svec_t encodeBytes() const;
    virtual void encodedBytes(svec_t const&);

    virtual uint32_t serializeQuality() const;
    virtual void serializeQuality(uint32_t);

    virtual uint32_t channels() const;
    virtual void channels(uint32_t);

    protected:
    std::string m_compress_fmt;
    TimeStamp m_ts;
    UID m_uid;
    uint32_t m_quality;
    mutable uint32_t m_channels;
    svec_t m_bytes;
    svec_t m_private_bytes;
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
