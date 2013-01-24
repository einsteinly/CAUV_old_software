/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_SONAR_ACCUMULATOR_H__
#define __CAUV_SONAR_ACCUMULATOR_H__

#include <boost/shared_ptr.hpp>

#include <stdint.h>

#include "trig.h"

namespace cv{
class Mat;
}

namespace cauv{

struct SonarDataLine;
struct PolarImage;
struct NonUniformPolarMat;
class Image;

float msgPolarAngleToRadians(int32_t angle);

class SonarAccumulator
{
    public: 
        SonarAccumulator(uint32_t size=400);
        
        bool accumulateDataLine(const SonarDataLine& data);
        bool setWholeImage(PolarImage const& image);
        bool setWholeImage(NonUniformPolarMat image);

        void setSize(uint32_t size);

        boost::shared_ptr<Image> img() const;
        cv::Mat mat() const;

    protected:

        int m_last_line_bearing;    
        float m_image_completed;
        boost::shared_ptr<Image> m_img;
        int m_bearingRange, m_range, m_scanWidth;
        size_t m_nbins;
        // diameter of accumulator 
        uint32_t m_size;

        // cos/sin caches:
        GemCachedTrig gem_cached;
        CachedTrig cached_trig;
        
        void reset();
};

} // namespace cauv

#endif //__CAUV_SONAR_ACCUMULATOR_H__

