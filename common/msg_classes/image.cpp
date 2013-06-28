/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "image.h"

#include <algorithm>

#include <boost/make_shared.hpp> 
#include <boost/variant/apply_visitor.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <debug/cauv_debug.h>
#include <utility/serialisation.h>
#include <utility/foreach.h>
#include <utility/rounding.h>
#include <generated/types/TimeStamp.h>
#include <generated/types/UID.h>

const static int Compress_JPEG_Quality = 95;

cauv::NonUniformPolarMat cauv::NonUniformPolarMat::clone() const{
    NonUniformPolarMat r;
    r.mat = mat.clone();
    r.ranges = boost::make_shared< std::vector<float> >(*ranges);
    r.bearings = boost::make_shared< std::vector<float> >(*bearings);
    return r;
}
cv::Point2f cauv::NonUniformPolarMat::xyAt(float r, float phi) const {
    float range, bearing;
    return xyAt(r,phi,range,bearing);
}
cv::Point2f cauv::NonUniformPolarMat::xyAt(float r, float phi, float& range, float& bearing) const {
    int r_r = clamp(0, std::floor(0.5+r), ranges->size()-1);
    int phi_r = clamp(0, std::floor(0.5+phi), bearings->size()-1);
    // TODO: interpolate?
    return xyAt(r_r, phi_r, range, bearing);
}
cv::Point2f cauv::NonUniformPolarMat::xyAt(int r, int phi) const {
    float range, bearing;
    return xyAt(r,phi,range,bearing);
}
cv::Point2f cauv::NonUniformPolarMat::xyAt(int r, int phi, float& range, float& bearing) const {
    range = ranges->at(r);
    bearing = bearings->at(phi);
    // x increases along the zero-bearing axis, y decreases
    // with increasing bearing in the first quadrant
    // (mathematical angles)
    // the bearings are in radians (see SonarInputNode)
    return cv::Point2f(range*cos(bearing), -range*sin(bearing));
}

cauv::PyramidMat cauv::PyramidMat::clone() const{
    PyramidMat r;
    r.levels.reserve(levels.size());
    foreach(cv::Mat m, levels)
        r.levels.push_back(m.clone());
    return r;
}

namespace cauv{
struct clone: boost::static_visitor<cauv::augmented_mat_t>{
    template <typename T>
    cauv::augmented_mat_t operator()(T const& r) const{
        return r.clone();
    }
};
// used for serialising
// !!! TODO: serialise everything
struct getPrincipalMat: boost::static_visitor<cv::Mat>{
    cv::Mat operator()(cv::Mat a) const{
        return a;
    }
    cv::Mat operator()(NonUniformPolarMat a) const{
        return a.mat;
    }
    cv::Mat operator()(PyramidMat a) const{
        if(a.levels.size())
            return a.levels[0];
        else
            return cv::Mat();
    }
};
struct getImageSizeInBits: boost::static_visitor<float>{
    float operator()(cv::Mat a) const{
        return a.rows * a.cols * a.elemSize() * 8;
    }
    float operator()(NonUniformPolarMat a) const{
        return a.mat.rows * a.mat.cols * a.mat.elemSize() * 8; 
    }
    float operator()(PyramidMat a) const{
        float r = 0;
        foreach(cv::Mat const& m, a.levels)
            r += operator()(m);
        return r;
    }
};
} // namespace cauv

cauv::Image::Image()
    : m_img() 
{
}

cauv::Image::Image(augmented_mat_t const& img)
    : m_img(img)
{
}

cauv::Image::Image(augmented_mat_t const& img, TimeStamp const& ts)
    : BaseImage(svec_t(), ts), m_img(img) 
{
}

cauv::Image::Image(augmented_mat_t const& img, TimeStamp const& ts, UID const& id)
    : BaseImage(svec_t(), ts, id), m_img(img) 
{
}

// Copy constructor; take a deep copy of the image data, make sure to copy the
// UID too.
cauv::Image::Image(Image const& other)
    : BaseImage(svec_t(), other.ts(), other.id()), 
      m_img(boost::apply_visitor(cauv::clone(), other.m_img))
{
}

// deep copy
cauv::Image& cauv::Image::operator=(Image const& other) {
    m_img = boost::apply_visitor(cauv::clone(), other.m_img);
    m_ts = other.m_ts;
    m_compress_fmt = other.m_compress_fmt;
    return *this;
}

cauv::Image::~Image(){
}


cv::Mat cauv::Image::mat() const {
    // will throw if this isn't the right type
    return boost::get<cv::Mat>(m_img);
}

void cauv::Image::mat(cv::Mat const& mat) {
    // will wipe out any augmented stuff
    m_img = mat;
}


cauv::augmented_mat_t cauv::Image::augmentedMat() const{
    return m_img;
}

void cauv::Image::augmentedMat(augmented_mat_t const& m){
    m_img = m;
}

float cauv::Image::bits() const{
    return boost::apply_visitor(getImageSizeInBits(), m_img);
}

uint32_t cauv::Image::channels() const {
    if(boost::apply_visitor(getPrincipalMat(), m_img).channels() == 1) {
        m_channels = 1;
    } else {
        m_channels = 3;
    }
    return m_channels;
}

cauv::svec_t cauv::Image::encodeBytes() const {
    // !!! TODO: serialise augmented data
    cv::Mat source = boost::apply_visitor(getPrincipalMat(), m_img);
    cv::Mat converted;
    std::vector<int> compress_params;
    compress_params.push_back(cv::IMWRITE_JPEG_QUALITY);
    compress_params.push_back(serializeQuality());

    if (source.depth() != CV_8U) {
        if (source.depth() == CV_32F || source.depth() == CV_64F)
            source.convertTo(converted, CV_8U, 255.0, 0.0);
        else
            source.convertTo(converted, CV_8U);
    }
    else
        converted = source;
    
    svec_t r;
    r.reserve(converted.cols * converted.rows * converted.elemSize() * 0.2);
    try {
        cv::imencode(m_compress_fmt, converted, r, compress_params);
    } catch(cv::Exception& e) {
        error() << "OpenCV couldn't encode image:"
                << "error:" << e.msg
                << "format:" << m_compress_fmt 
                << "quality:" << serializeQuality()
                << "size:" << source.rows << "x" << source.cols;
    }
    return r;
}

void cauv::Image::encodedBytes(svec_t const& bytes) {
    int load_flags;
    if (m_channels == 3) {
        load_flags = 1;
    } else if (m_channels == 1) {
        load_flags = 0;
    } else {
        error() << "Tried to decode image with" << channels() << "channels!";
        return;
    } 
    try {
        m_img = cv::imdecode(cv::Mat(bytes), load_flags);
    } catch (cv::Exception &e) {
        error() << "OpenCV couldn't decode image:"
                << "error:" << e.msg
                << "format:" << m_compress_fmt
                << "channels:" << channels();
    }
}


#if 0
void serialise(svec_ptr p, Image const& v) {
    serialise(p, v.m_compress_fmt);
    serialise(p, v.m_compress_params);
    int32_t load_flags = -1;
    
    cv::Mat source = boost::apply_visitor(getPrincipalMat(), v.m_img);
    cv::Mat converted;
    switch(source.channels()){
        case 1:
            load_flags = 0;
            if (source.type() != CV_8UC1)
                source.convertTo(converted, CV_8UC1);
            else
                converted = source;
            break;
        case 3:
        default:
            load_flags = 1;
            if (source.type() != CV_8UC3)
                source.convertTo(converted, CV_8UC3);
            else
                converted = source;
            break;
    }
    serialise(p, load_flags);

    std::vector<uint8_t> buf;
    buf.reserve(converted.cols * converted.rows * converted.elemSize() * 0.2);
    try{
        cv::imencode(v.m_compress_fmt, converted, buf, v.m_compress_params);
    }catch(cv::Exception& e){
        error() << "OpenCV couldn't encode image:"
                << "format:" << v.m_compress_fmt 
                << "params:" << v.m_compress_params
                << "size:" << source.rows << "x" << source.cols;
    }
    // rely on efficient overload of serialise for vector<uint8_t>
    p->reserve(p->size() + buf.size() + sizeof(v.m_ts));
    serialise(p, buf);
    serialise(p, v.m_ts);
    
    //const float pre = source.rows * source.cols * source.elemSize();
    //const float post = buf.size();
    //std::cout << "Image Serialization:\n\t"
    //          << __func__ << m_compress_fmt << m_compress_params << load_flags
    //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
}

int32_t deserialise(const_svec_ptr p, uint32_t i, Image& v){
    uint32_t b = i;
    b += deserialise(p, b, v.m_compress_fmt);
    b += deserialise(p, b, v.m_compress_params);

    int32_t load_flags = -1;
    b += deserialise(p, b, load_flags);

    std::vector<uint8_t> buf;
    b += deserialise(p, b, buf);

    try{
        v.m_img = cv::imdecode(cv::Mat(buf), load_flags);
    }catch(cv::Exception& e){
        error() << "OpenCV couldn't decode image:"
                << "format:" << v.m_compress_fmt 
                << "params:" << v.m_compress_params;
    }
    
    b += deserialise(p, b, v.m_ts);
    
    //cv::Mat source = boost::get<cv::Mat>(v.m_img);
    //const float post = source.rows * source.cols * source.elemSize();
    //const float pre = buf.size();
    //std::cout << "Image Deserialization:\n\t"
    //          << __func__ << m_compress_fmt << m_compress_params << load_flags
    //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
    return b - i;
}
#endif
