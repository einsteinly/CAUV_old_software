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
        
        void accumulateDataLine(const SonarDataLine& data);

        boost::shared_ptr<Image> img() const;
        cv::Mat const& mat() const;

    protected:
        boost::shared_ptr<Image> m_img;
};

} // namespace cauv

#endif //__CAUV_SONAR_ACCUMULATOR_H__

