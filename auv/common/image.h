#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <boost/serialization/split_member.hpp>
#include <boost/serialization/level.hpp>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <common/streamops.h>
#include <debug/cauv_debug.h>

class Image{
        friend class boost::serialization::access;
    public:
        Image();
        Image(cv::Mat const& cv_image);
        Image(Image const& other);
        ~Image();

        cv::Mat const& cvMat() const;
        cv::Mat& cvMat();

        void serializeQuality(int);

        BOOST_SERIALIZATION_SPLIT_MEMBER()

        template<class Archive>
        void save(Archive& ar, const unsigned int /*version*/) const{
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
            try{
                cv::imencode(m_compress_fmt, m_img, buf, m_compress_params);
            }catch(cv::Exception& e){
                error() << "OpenCV couldn't encode image:"
                        << "format:" << m_compress_fmt 
                        << "params:" << m_compress_params
                        << "size:" << m_img.rows << "x" << m_img.cols;
            }
            ar & buf;
            
            //const float pre = m_img.rows * m_img.cols * m_img.elemSize();
            //const float post = buf.size();
            //std::cout << "Image Serialization:\n\t"
            //          << __func__ << m_compress_fmt << m_compress_params << load_flags
            //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
        }
        
        template<class Archive>
        void load(Archive& ar, const unsigned int /*version*/){
            ar & m_compress_fmt;
            ar & m_compress_params;
            int load_flags = -1;
            ar & load_flags;

            std::vector<unsigned char> buf;
            ar & buf;
            try{
                m_img = cv::imdecode(cv::Mat(buf), load_flags);
            }catch(cv::Exception& e){
                error() << "OpenCV couldn't decode image:"
                        << "format:" << m_compress_fmt 
                        << "params:" << m_compress_params;
            }

            //const float post = m_img.rows * m_img.cols * m_img.elemSize();
            //const float pre = buf.size();
            //std::cout << "Image Deserialization:\n\t"
            //          << __func__ << m_compress_fmt << m_compress_params << load_flags
            //          << "(" << pre << "->" << post << "bytes = " << post / pre << ")";
        }

    private:
        cv::Mat m_img;

        std::string m_compress_fmt;
        std::vector<int> m_compress_params;
};
BOOST_CLASS_IMPLEMENTATION(Image, boost::serialization::object_serializable)

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
    os << "{Image mat=" << img.cvMat() << "}";
    return os;
}

#endif // ndef __IMAGE_H__
