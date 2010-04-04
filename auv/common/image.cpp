#include "image.h"

const static int Compress_JPEG_Quality = 90;

Image::Image()
    : m_img(), m_source(), m_compress_fmt(".jpg"), m_compress_params(){
    m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_compress_params.push_back(Compress_JPEG_Quality);
}

Image::Image(cv::Mat const& cv_image, Source const& source)
    : m_img(cv_image), m_source(source), m_compress_fmt(".jpg"),
    m_compress_params(){
    m_compress_params.push_back(CV_IMWRITE_JPEG_QUALITY);
    m_compress_params.push_back(Compress_JPEG_Quality);
}

// Copy constructor; take a deep copy of the image data
Image::Image(Image const& other)
    : m_img(other.m_img.clone()), m_source(other.m_source),
      m_compress_fmt(other.m_compress_fmt),
      m_compress_params(other.m_compress_params){
}

Image::~Image(){
}

cv::Mat const& Image::cvMat() const{
    return m_img;
}

cv::Mat& Image::cvMat(){
    return m_img;
}

Image::Source Image::source() const{
    return m_source;
}

void Image::source(Source const& s){
    m_source = s;
}

