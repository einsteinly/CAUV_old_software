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

#include "image.h"

#include <algorithm>

#include <boost/make_shared.hpp> 
#include <boost/variant/apply_visitor.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <utility/serialisation.h>

const static int Compress_JPEG_Quality = 95;

cauv::NonUniformPolarMat cauv::NonUniformPolarMat::clone() const{
    NonUniformPolarMat r;
    r.mat = mat.clone();
    r.ranges = boost::make_shared< std::vector<float> >(*ranges);
    r.bearings = boost::make_shared< std::vector<float> >(*bearings);
    return r;
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
} // namespace cauv

cauv::Image::Image()
    : m_img(), m_ts(now()), m_compress_fmt(".jpg"), m_compress_params(), m_uid(mkUID())
{
    setDefaultCompressParams();
}

cauv::Image::Image(augmented_mat_t const& img)
    : m_img(img), m_ts(now()), m_compress_fmt(".jpg"), m_compress_params(), m_uid(mkUID())
{
    setDefaultCompressParams();
}

cauv::Image::Image(augmented_mat_t const& img, TimeStamp const& ts)
    : m_img(img), m_ts(ts), m_compress_fmt(".jpg"), m_compress_params(), m_uid(mkUID())
{
    setDefaultCompressParams();
}

cauv::Image::Image(augmented_mat_t const& img, TimeStamp const& ts, UID const& id)
    : m_img(img), m_ts(ts), m_compress_fmt(".jpg"), m_compress_params(), m_uid(id)
{
    setDefaultCompressParams();
}

// Copy constructor; take a deep copy of the image data
cauv::Image::Image(Image const& other)
    : m_img(boost::apply_visitor(cauv::clone(), other.m_img)),
       m_ts(other.m_ts), 
      m_compress_fmt(other.m_compress_fmt),
      m_compress_params(other.m_compress_params){
}

// deep copy
cauv::Image& cauv::Image::operator=(Image const& other){
    m_img = boost::apply_visitor(cauv::clone(), other.m_img);
    m_ts = other.m_ts;
    m_compress_fmt = other.m_compress_fmt;
    m_compress_params = other.m_compress_params;
    return *this;
}

cauv::Image::~Image(){
}


cauv::TimeStamp cauv::Image::ts() const{
    return m_ts;
}

void cauv::Image::ts(cauv::TimeStamp const& t){
    m_ts = t;
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

void cauv::Image::serializeQuality(int q){
    std::vector<int>::iterator i = std::find(m_compress_params.begin(),
                                             m_compress_params.end(),
                                             int(CV_IMWRITE_JPEG_QUALITY));
    if(i != m_compress_params.end()){
        if(++i != m_compress_params.end())
            *i = q;
        else
            m_compress_params.push_back(q);
    }else{
        m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        m_compress_params.push_back(q);
    }
}

void cauv::Image::setDefaultCompressParams(){
    m_compress_params.clear();
    m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_compress_params.push_back(Compress_JPEG_Quality);
}

void cauv::serialise(svec_ptr p, Image const& v){
    serialise(p, v.m_compress_fmt);
    serialise(p, v.m_compress_params);
    int32_t load_flags = -1;
    
    // !!! TODO: serialise augmented data
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
    try{
        cv::imencode(v.m_compress_fmt, converted, buf, v.m_compress_params);
    }catch(cv::Exception& e){
        error() << "OpenCV couldn't encode image:"
                << "format:" << v.m_compress_fmt 
                << "params:" << v.m_compress_params
                << "size:" << source.rows << "x" << source.cols;
    }
    // rely on efficient overload of serialise for vector<uint8_t>
    serialise(p, buf);
    serialise(p, v.m_ts);
    
    //const float pre = source.rows * source.cols * source.elemSize();
    //const float post = buf.size();
    //std::cout << "Image Serialization:\n\t"
    //          << __func__ << m_compress_fmt << m_compress_params << load_flags
    //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
}
std::string cauv::chil(Image const& v){
    svec_ptr p = boost::make_shared<svec_t>();
    serialise(p, v);
    return cauv::chil(*p);
} 

int32_t cauv::deserialise(const_svec_ptr p, uint32_t i, Image& v){
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
