#include "image.h"

#include <algorithm>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <debug/cauv_debug.h>
#include <utility/serialisation.h>

using namespace cauv;

const static int Compress_JPEG_Quality = 95;

Image::Image()
    : m_img(new cv::Mat()), m_compress_fmt(".jpg"), m_compress_params()
{
    m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_compress_params.push_back(Compress_JPEG_Quality);
}

Image::Image(cv::Mat const& cv_image)
    : m_img(new cv::Mat(cv_image)), m_compress_fmt(".jpg"), m_compress_params()
{
    m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_compress_params.push_back(Compress_JPEG_Quality);
}

// Copy constructor; take a deep copy of the image data
Image::Image(Image const& other)
    : m_img(new cv::Mat(other.m_img->clone())),
      m_compress_fmt(other.m_compress_fmt),
      m_compress_params(other.m_compress_params){
}

// deep copy
Image& Image::operator=(Image const& other){
    m_img.reset(new cv::Mat(other.m_img->clone()));
    m_compress_fmt = other.m_compress_fmt;
    m_compress_params = other.m_compress_params;
    return *this;
}

Image::~Image(){
}

cv::Mat const& Image::cvMat() const{
    return *m_img;
}

cv::Mat& Image::cvMat(){
    return *m_img;
}

void Image::serializeQuality(int q){
    std::vector<int>::iterator i;
    if((i = std::find(
           m_compress_params.begin(), m_compress_params.end(), int(CV_IMWRITE_JPEG_QUALITY)
       )) != m_compress_params.end()){
        if(++i != m_compress_params.end())
            *i = q;
        else
            m_compress_params.push_back(q);
    }else{
        m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
        m_compress_params.push_back(q);
    }
}

void cauv::serialise(svec_ptr p, Image const& v){
    serialise(p, v.m_compress_fmt);
    serialise(p, v.m_compress_params);
    int32_t load_flags = -1;

    cv::Mat converted;
    switch(v.m_img->channels()){
        case 1:
            load_flags = 0;
            if (v.m_img->type() != CV_8UC1)
                v.m_img->convertTo(converted, CV_8UC1);
            else
                converted = *v.m_img;
            break;
        case 3:
        default:
            load_flags = 1;
            if (v.m_img->type() != CV_8UC3)
                v.m_img->convertTo(converted, CV_8UC3);
            else
                converted = *v.m_img;
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
                << "size:" << v.m_img->rows << "x" << v.m_img->cols;
    }
    // rely on efficient overload of serialise for vector<uint8_t>
    serialise(p, buf);
    
    //const float pre = v.m_img->rows * v.m_img->cols * v.m_img->elemSize();
    //const float post = buf.size();
    //std::cout << "Image Serialization:\n\t"
    //          << __func__ << m_compress_fmt << m_compress_params << load_flags
    //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
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
        *v.m_img = cv::imdecode(cv::Mat(buf), load_flags);
    }catch(cv::Exception& e){
        error() << "OpenCV couldn't decode image:"
                << "format:" << v.m_compress_fmt 
                << "params:" << v.m_compress_params;
    }

    //const float post = v.m_img->rows * v.m_img->cols * v.m_img->elemSize();
    //const float pre = buf.size();
    //std::cout << "Image Deserialization:\n\t"
    //          << __func__ << m_compress_fmt << m_compress_params << load_flags
    //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
    return b - i;
}

