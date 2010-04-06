#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <boost/serialization/split_member.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/debug.h>

class Image{
        friend class boost::serialization::access;
    public:
        enum Source {
            src_file,
            src_camera,
            src_sonar,
            src_pipeline
        };
        
        Image();
        Image(cv::Mat const& cv_image, Source const& source);
        Image(Image const& other);
        ~Image();

        cv::Mat const& cvMat() const;
        cv::Mat& cvMat();
        Source source() const;
        void source(Source const& s);

        void serializeQuality(int);

        BOOST_SERIALIZATION_SPLIT_MEMBER()

        template<class Archive>
        void save(Archive& ar, const unsigned int /*version*/) const{
            ar & m_source;
            ar & m_compress_fmt;
            ar & m_compress_params;
            int load_flags = -1;
            switch(m_img.channels()){
                case 1: load_flags = 0; break;
                case 3: load_flags = 1; break;
                case 4: default: load_flags = -1; break;
            }
            ar & load_flags;

            std::vector<unsigned char> buf;
            cv::imencode(m_compress_fmt, m_img, buf, m_compress_params);
            ar & buf;

            debug() << "Image Serialization:\n\t"
                      << __func__ << m_compress_fmt << m_compress_params << "("
                      << m_img.rows * m_img.cols * m_img.elemSize() << "->"
                      << buf.size() << "bytes)";
        }
        
        template<class Archive>
        void load(Archive& ar, const unsigned int /*version*/){
            ar & m_source;
            ar & m_compress_fmt;
            ar & m_compress_params;
            int load_flags = -1;
            ar & load_flags;

            std::vector<unsigned char> buf;
            ar & buf;
            m_img = cv::imdecode(cv::Mat(buf), load_flags);

            debug() << "Image Serialization:\n\t"
                      << __func__ << m_compress_fmt << m_compress_params << "("
                      << buf.size() << "->"
                      << m_img.rows * m_img.cols * m_img.elemSize() << "bytes)";
        }

    private:
        cv::Mat m_img;

        Source m_source;
        std::string m_compress_fmt;
        std::vector<int> m_compress_params;
};

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image::Source const& s){
    switch(s){
        case Image::src_file: os << "file"; break;
        case Image::src_camera: os << "camera"; break;
        case Image::src_sonar: os << "sonar"; break;
        case Image::src_pipeline: os << "pipeline"; break;        
    }
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, cv::Mat const& m){
    os << "{Mat s=" << m.size().width << "x" << m.size().height
       << " flags=" << std::hex << m.flags << "}";
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image const& img){
    os << "{Image src=" << img.source() << " mat=" << img.cvMat() << "}";
    return os;
}

#endif // ndef __IMAGE_H__
