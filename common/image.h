#ifndef __CAUV_IMAGE_H__
#define __CAUV_IMAGE_H__

#include <boost/scoped_ptr.hpp>

#include <utility/streamops.h>
#include <utility/serialisation-types.h>

namespace cv{
class Mat;
}

namespace cauv{

class Image{
        friend void cauv::serialise(svec_ptr, Image const&);
        friend int32_t cauv::deserialise(const_svec_ptr, uint32_t i, Image&);
    public:
        Image();
        Image(cv::Mat const& cv_image);
        Image(Image const& other);
        Image& operator=(Image const& other);
        ~Image();

        cv::Mat const& cvMat() const;
        cv::Mat& cvMat();

        void serializeQuality(int32_t);

    private:
        boost::scoped_ptr<cv::Mat> m_img;

        std::string m_compress_fmt;
        std::vector<int32_t> m_compress_params;
};

/* image serialisation */
void serialise(svec_ptr p, Image const& v);
int32_t deserialise(const_svec_ptr p, uint32_t i, Image& v);

template<typename charT, typename traits>
std::basic_ostream<charT, traits>& operator<<(
    std::basic_ostream<charT, traits>& os, Image const&){
    os << "{Image}";
    return os;
}

} // namespace cauv

#endif // ndef __CAUV_IMAGE_H__
