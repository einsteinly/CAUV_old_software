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
        
        bool accumulateDataLine(const SonarDataLine& data);

        boost::shared_ptr<Image> img() const;
        cv::Mat mat() const;

    protected:
        int m_last_line_bearing;    
        float m_image_completed;
        boost::shared_ptr<Image> m_img;
        int m_bearingRange, m_range, m_scanWidth;
        size_t m_nbins;
        
        void reset();
};

} // namespace cauv

#endif //__CAUV_SONAR_ACCUMULATOR_H__

