#ifndef __SONAR_ACCUMULATOR_H__
#define __SONAR_ACCUMULATOR_H__

#include <boost/shared_ptr.hpp>

class SonarDataLine;
class Image;

namespace cv
{
class Mat;
}


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


#endif //__SONAR_ACCUMULATOR_H__
