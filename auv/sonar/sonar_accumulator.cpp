#include "display_sonar_observer.h"

#include <cmath>
#include <algorithm>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>

#include <generated/messages.h>
#include <debug/cauv_debug.h>

using namespace std;
using namespace cauv;

static inline cv::Rect arcBound(int radius, cv::Point2f from, cv::Point2f to)
{
    // Assuming maximum quarter arcs

    // Check which half-axes arc crosses, by checking quadrants

    // 10 | 00
    // ___|___
    //    |
    // 11 | 01

    int8_t fromQuad = ((from.x < 0) << 1) | (from.y < 0);
    int8_t toQuad = ((to.x < 0) << 1) | (to.y < 0);
    
    int max_x, max_y, min_x, min_y;

    if (fromQuad == toQuad) // Common case, same quadrant
    {
		min_x = floor(min(from.x, to.x));
		min_y = floor(min(from.y, to.y));
		max_x = ceil(max(from.x, to.x));
		max_y = ceil(max(from.y, to.y));
    }
    else
    {
        switch (fromQuad)
        {
            default:
                error() << "fromQuad out of range:" << fromQuad;
            case 0:
		        min_x = to.x;
                min_y = floor(min(from.y, to.y));
		        max_x = from.x;
		        max_y = radius;
                break;
            case 1:
                min_x = floor(min(from.x, to.x));
                min_y = from.y;
                max_x = radius;
                max_y = to.y;
                break;
            case 2:
                min_x = -radius;
                min_y = to.y;
                max_x = ceil(max(from.x, to.x));
                max_y = from.y;
                break;
            case 3:
                min_x = from.x;
                min_y = -radius;
                max_x = to.x;
                max_y = ceil(max(from.y, to.y));
                break;
        }
    }
    return cv::Rect(min_x, min_y, max_x - min_x + 1, max_y - min_y + 1);
}

template<typename T>
inline bool ccw(T p1_x, T p1_y, T p2_x, T p2_y)
{
    return (p1_x * p2_y - p2_x * p1_y) > 0;
}


SonarAccumulator::SonarAccumulator()
    : m_last_line_bearing(0),
      m_images_accumulated(0),
      m_img(boost::make_shared<Image>(cv::Mat::zeros(400,400,CV_8UC1)))
{   
    assert(m_img->mat().data);
}


std::vector<float> precalculate_sonar_sins()
{
    std::vector<float> ret;
    ret.reserve(1600);
    for (int b = 0; b < 1600; ++b)
        ret.push_back(sin((M_PI / 3200) * b));
    return ret;
}
static std::vector<float> sonar_sins = precalculate_sonar_sins();

float sonar_sin(int bearing)
{
    bearing = bearing % 6400;
    if (bearing < 0)
        bearing += 6400;
    
    if (bearing < 1600)
        return sonar_sins[bearing];
    else if (bearing < 3200)
        return sonar_sins[3200 - bearing];
    else if (bearing < 4800)
        return -sonar_sins[bearing - 3200];
    else
        return -sonar_sins[6400 - bearing];   
}
float sonar_cos(int bearing)
{
    return sonar_sin(bearing + 1600);
}


float SonarAccumulator::accumulateDataLine(const SonarDataLine& line)
{
    if (line.bearingRange > 1600)
    {
        error() << "Cannot deal with arcs larger than a quarter circle. Do you really need the range to be this coarse?";
        return m_images_accumulated;
    }

    int from = line.bearing - line.bearingRange/2;
    int to = from + line.bearingRange;

    int radius = floor((min(m_img->height(), m_img->width())-1)/2);
    int bincount = line.data.size();

    float bscale = (float)radius/bincount;
    float cx = radius, cy = radius;
    
    int accumulated_delta = line.bearing - m_last_line_bearing;
    m_last_line_bearing = line.bearing;
    if (accumulated_delta < 0)
        m_images_accumulated += 1.0;
    m_images_accumulated += double(accumulated_delta) / 6400.0;

    cv::Mat m = m_img->mat();

    for (int b = 0; b < bincount; b++) {
        // All calculations assume centre is at (0,0)
        
        float inner_radius = b * bscale;
        float outer_radius = (b+1) * bscale;

        cv::Point2f pt_inner_from(inner_radius*sonar_cos(from), inner_radius*sonar_sin(from));
        cv::Point2f pt_inner_to(inner_radius*sonar_cos(to), inner_radius*sonar_sin(to));
        cv::Point2f pt_outer_from(outer_radius*sonar_cos(from), outer_radius*sonar_sin(from));
        cv::Point2f pt_outer_to(outer_radius*sonar_cos(to), outer_radius*sonar_sin(to));
        
        cv::Rect innerbb = arcBound(inner_radius, pt_inner_from, pt_inner_to);
        cv::Rect outerbb = arcBound(outer_radius, pt_outer_from, pt_outer_to);
        cv::Rect bb = innerbb | outerbb;
        
        for(int y = bb.y; y < bb.y + bb.height; y++)
        {
            unsigned char* pm = &m.at<unsigned char>(cy - y, cx + bb.x);
            for(int x = bb.x; x < bb.x + bb.width; x++, pm++)
            {
                int r2 = x*x + y*y;
        
                // Check if we're at least within the right radius
                if (r2 < inner_radius * inner_radius || r2 > outer_radius * outer_radius)
                    continue;
                
                //  | P /  For P is inside the segment if
                //  |  /   it's ccw of from and cw of to.                 
                //  | /
                //  |/     
                bool ccwFrom = ccw<float>(pt_outer_from.x, pt_outer_from.y, x, y);
                bool ccwTo = ccw<float>(pt_outer_to.x, pt_outer_to.y, x, y);
                
                if (!ccwFrom || ccwTo)
                    continue;

                // If we've got to here, then this must be valid. Shift to the centre and set the value.
                *pm = line.data[b];
            }
        }
    }
    m_img->mat(m);

    return m_images_accumulated;
}



boost::shared_ptr<Image> SonarAccumulator::img() const
{
    return m_img;
}

cv::Mat SonarAccumulator::mat() const
{
    return m_img->mat();
}


