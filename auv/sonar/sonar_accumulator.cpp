#include "display_sonar_observer.h"

#include <cmath>
#include <algorithm>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <opencv/cv.h>

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
bool ccw(T p1_x, T p1_y, T p2_x, T p2_y)
{
    return (p1_x * p2_y - p2_x * p1_y) > 0;
}


SonarAccumulator::SonarAccumulator()
    : m_img(boost::make_shared<Image>(cv::Mat::zeros(400,400,CV_8UC1)))
{   
    assert(m_img->cvMat().data);
}
void SonarAccumulator::accumulateDataLine(const SonarDataLine& line)
{
    if (line.bearingRange > 1600)
    {
        error() << "Cannot deal with arcs larger than a quarter circle. Do you really need the range to be this coarse?";
        return;
    }

    int bearing_from = line.bearing - line.bearingRange/2 - 200;
    int bearing_to = bearing_from + line.bearingRange;

    float a_from = (M_PI / 3200) * bearing_from;
	float a_to = (M_PI / 3200) * bearing_to;

    // Precalculate expensive trig
    float cosfrom = cos(a_from);
    float sinfrom = sin(a_from);
    float costo = cos(a_to);    
    float sinto = sin(a_to);    

    int radius = floor((min(m_img->cvMat().rows, m_img->cvMat().cols)-1)/2);
    int bincount = line.data.size();

    float bscale = (float)radius/bincount;
    float cx = radius, cy = radius;

    for (int b = 0; b < bincount; b++) {
        // All calculations assume centre is at (0,0)
        
        float inner_radius = b * bscale;
        float outer_radius = (b+1) * bscale;

        cv::Point2f pt_inner_from(inner_radius*cosfrom, inner_radius*sinfrom);
        cv::Point2f pt_inner_to(inner_radius*costo, inner_radius*sinto);
        cv::Point2f pt_outer_from(outer_radius*cosfrom, outer_radius*sinfrom);
        cv::Point2f pt_outer_to(outer_radius*costo, outer_radius*sinto);
        
        cv::Rect innerbb = arcBound(inner_radius, pt_inner_from, pt_inner_to);
        cv::Rect outerbb = arcBound(outer_radius, pt_outer_from, pt_outer_to);
        cv::Rect bb = innerbb | outerbb;
        
        for(int y = bb.y; y < bb.y + bb.height; y++)
            for(int x = bb.x; x < bb.x + bb.width; x++)
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
                m_img->cvMat().at<unsigned char>(cy - y, cx + x) = line.data[b];
            }
    }
}



boost::shared_ptr<Image> SonarAccumulator::img() const
{
    return m_img;
}

cv::Mat const& SonarAccumulator::mat() const
{
    return m_img->cvMat();
}


