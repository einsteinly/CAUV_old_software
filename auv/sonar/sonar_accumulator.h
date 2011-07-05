#ifndef __CAUV_SONAR_ACCUMULATOR_H__
#define __CAUV_SONAR_ACCUMULATOR_H__

#include <boost/shared_ptr.hpp>

namespace cv{
class Mat;
}

namespace cauv{

struct SonarDataLine;
class Image;

class SonarAccumulator
{
    public: 
        SonarAccumulator();
        
        float accumulateDataLine(const SonarDataLine& data);

        boost::shared_ptr<Image> img() const;
        cv::Mat mat() const;

    protected:
        int m_last_line_bearing;    
        double m_images_accumulated;
        boost::shared_ptr<Image> m_img;
};

} // namespace cauv

#endif //__CAUV_SONAR_ACCUMULATOR_H__

