#ifndef __CAUV_IMAGE_H__
#define __CAUV_IMAGE_H__

#include <boost/cstdint.hpp>

#include <opencv2/core/core.hpp>

#include <utility/streamops.h>
#include <utility/serialisation-types.h>

namespace cauv{

class Image{
        friend void serialise(svec_ptr, Image const&);
        friend int32_t deserialise(const_svec_ptr, uint32_t i, Image&);
    public:
        Image();
        Image(cv::Mat const& cv_image);
        Image(Image const& other);
        Image& operator=(Image const& other);
        ~Image();

        cv::Mat mat() const;
        void mat(cv::Mat const& mat);

        int width() const;
        int height() const;
        int channels() const;
        int type() const;
        int depth() const;
        cv::Size size() const;
        bool empty() const;

        void serializeQuality(int32_t);

    private:
        cv::Mat m_img;

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
