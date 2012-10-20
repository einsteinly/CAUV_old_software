#include "base_image.h"
#include <utility/serialisation.h>
#include <boost/make_shared.hpp>

using namespace cauv;

const static int Default_Compress_JPEG_Quality = 95;

BaseImage::BaseImage(void) :
    m_compress_fmt(".jpg"), m_ts(now()), m_uid(mkUID()),
    m_quality(Default_Compress_JPEG_Quality) {

}

BaseImage::BaseImage(svec_t const &bytes) :
    m_compress_fmt(".jpg"), m_ts(now()), m_uid(mkUID()),
    m_quality(Default_Compress_JPEG_Quality),
    m_bytes(bytes) {

}

BaseImage::BaseImage(svec_t const &bytes, TimeStamp const &ts) :
    m_compress_fmt(".jpg"), m_ts(ts), m_uid(mkUID()),
    m_quality(Default_Compress_JPEG_Quality),
    m_bytes(bytes) {

}

BaseImage::BaseImage(svec_t const &bytes, TimeStamp const &ts, UID const &id) :
    m_compress_fmt(".jpg"), m_ts(ts), m_uid(id),
    m_quality(Default_Compress_JPEG_Quality),
    m_bytes(bytes) {

}

BaseImage::~BaseImage(void) {}

TimeStamp BaseImage::ts(void) const{
    return m_ts;
}

void BaseImage::ts(TimeStamp const &ts) {
    m_ts = ts;
}

UID BaseImage::id(void) const{
    return m_uid;
}

void BaseImage::id(UID const &ts) {
    m_uid = ts;
}

svec_t const& BaseImage::encodedBytes() const {
    return m_bytes;
}

// this base class doesn't know how to serialise an image (derived class must
// do that)
svec_t BaseImage::encodeBytes() const{
    return svec_t();
}

void BaseImage::encodedBytes(svec_t const& bytes) {
    m_bytes = bytes;
}

svec_t const& BaseImage::private_bytes(void) const {
    return m_private_bytes;
}

void BaseImage::private_bytes(svec_t &bytes) {
    m_private_bytes = bytes;
}

uint32_t BaseImage::serializeQuality() const {
    return m_quality;
}

void BaseImage::serializeQuality(uint32_t quality) {
    m_quality = quality;
}

uint32_t BaseImage::channels() const {
    return m_channels;
}

void BaseImage::channels(uint32_t channels) {
    m_channels = channels;
}

void cauv::serialise(svec_ptr p, BaseImage const& v) {
    serialise(p, v.m_compress_fmt);
    serialise(p, v.serializeQuality());
    serialise(p, v.channels());
    serialise(p, v.encodeBytes());
    serialise(p, v.private_bytes());
    serialise(p, v.ts());
    serialise(p, v.id());
}

//do nothing
std::string cauv::chil(BaseImage const& v) {
    svec_ptr p = boost::make_shared<svec_t>();
    serialise(p, v);
    return cauv::chil(*p);
} 

int32_t cauv::deserialise(const_svec_ptr p, uint32_t i, BaseImage& v) {
    uint32_t b = i;
    b += deserialise(p, b, v.m_compress_fmt);
    uint32_t quality;
    b += deserialise(p, b, quality);
    v.serializeQuality(quality);
    uint32_t channels;
    b += deserialise(p, b, channels);
    v.channels(channels);

    svec_t buf;
    b += deserialise(p, b, buf);
    svec_t private_buf;
    b += deserialise(p, b, private_buf);
    v.private_bytes(private_buf);
    v.encodedBytes(buf);

    TimeStamp ts;
    b += deserialise(p, b, ts);
    v.ts(ts);

    UID id;
    b += deserialise(p, b, id);
    v.id(id);
    
    return b - i;
}
