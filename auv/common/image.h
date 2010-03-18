#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <boost/serialization/split_member.hpp>

#include <opencv/cv.h>

#include <common/debug.h>

class Image{
        friend class boost::serialization::access;
    public:
        enum Source {src_file, src_camera, src_sonar};
        
        Image()
            : m_img(), m_source(){
        }
        ~Image(){
            delete[] m_img.data;
        }

        Image(cv::Mat const& cv_image, Source const& source)
            : m_img(cv_image), m_source(source){
        }
        
        // Copy constructor; take a deep copy of the image data
        Image(Image const& other)
            : m_img(other.m_img.clone()), m_source(other.m_source){
        }

        cv::Mat const& cvMat() const{
            return m_img;
        }

        cv::Mat& cvMat(){
            return m_img;
        }

        Source source() const{
            return m_source;
        }

        BOOST_SERIALIZATION_SPLIT_MEMBER()

        template<class Archive>
        void save(Archive& ar, const unsigned int version) const{
            ar & m_source;
            
            int type = m_img.type();
            ar & type;
            ar & m_img.rows;
            ar & m_img.cols;

            debug(-1) << BashColour::Cyan << "Image Serialization:\n\t"
                      << __func__ << type << m_img.elemSize() << m_img.rows << m_img.cols;
            
            ar.save_binary(m_img.data, m_img.rows * m_img.cols * m_img.elemSize());
        }
        
        template<class Archive>
        void load(Archive& ar, const unsigned int version){
            ar & m_source;
            
            int type, rows, cols;
            ar & type; 
            ar & rows;
            ar & cols;
            
            int size = rows * cols * typeWidth(type);
            unsigned char* data = new unsigned char[size];
            
            ar.load_binary(data, size);

            debug(-1) << BashColour::Cyan << "Image Serialization:\n\t"
                      << __func__ << type << typeWidth(type) << rows << cols;

            m_img = cv::Mat(rows, cols, type, data);
        }
        
        int typeWidth(int cv_type) const{
            unsigned depth = 0;
            unsigned channels = 0;

            switch(cv_type & CV_MAT_DEPTH_MASK){
                case CV_8U:  case CV_8S:  depth = 1; break;
                case CV_16U: case CV_16S: depth = 2; break;
                case CV_32S: case CV_32F: depth = 4; break;
                case CV_64F: depth = 8; break;
                default:
                    error() << "unknown opencv pixal depth value"
                            << (cv_type & CV_MAT_DEPTH_MASK);
            }
            channels = (cv_type >> CV_CN_SHIFT) + 1;
            return depth * channels;
        }
        
        
    private:
        cv::Mat m_img;
        Source m_source;
};

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image::Source const& s){
    switch(s){
        case Image::src_file: os << "file"; break;
        case Image::src_camera: os << "camera"; break;
        case Image::src_sonar: os << "sonar"; break;
    }
    return os;
}

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image const& img){
    os << "{Image src=" << img.source() << " dim="
       << img.cvMat().size().width << "x"
       << img.cvMat().size().height << "}";
    return os;
}

#endif // ndef __IMAGE_H__
