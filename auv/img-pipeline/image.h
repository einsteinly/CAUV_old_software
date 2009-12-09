#ifndef __IMAGE_H__
#define __IMAGE_H__

#include <opencv/cv.h>

class Image{
    public:
        enum Source {file, camera, sonar};
        
        Image(cv::Mat const& cv_image, Source const& source)
            : m_img(cv_image), m_source(source){
        }

        cv::Mat const& cvImg() const{
            return m_img;
        }
        
    private:
        cv::Mat m_img;
        Source m_source;
};

#endif // ndef __IMAGE_H__
