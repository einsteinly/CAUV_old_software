/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "sonar_accumulator.h"

#include <cmath>
#include <algorithm>

#include <boost/make_shared.hpp>
#include <boost/ref.hpp>

#include <opencv2/core/core.hpp>

#include <common/image.h>
#include <common/math.h>
#include <debug/cauv_debug.h>
#include <generated/types/SonarDataLine.h>
#include <generated/types/PolarImage.h>

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

// converts bearing (+ve clockwise) to angle (+ve anticlockwise)
float cauv::msgPolarAngleToRadians(int32_t bearing){
    return -M_PI * bearing / (3200.0*0x10000);
}

SonarAccumulator::SonarAccumulator(uint32_t size)
{   
    m_size = size;
    reset();
    assert(m_img->mat().data);
}

void SonarAccumulator::reset() {
    m_last_line_bearing = 0;
    m_image_completed = 0;
    m_img = boost::make_shared<Image>(cv::Mat::zeros(m_size,m_size,CV_8UC1));
}

bool SonarAccumulator::accumulateDataLine(const SonarDataLine& line)
{
    if (line.bearingRange != m_bearingRange ||
        line.range != m_range ||
        line.scanWidth != m_scanWidth)
    {
        // New sonar params, reset
        debug(3) << "Sonar params updates, resetting accumulator" << m_range << m_bearingRange << m_scanWidth << m_nbins;
        reset();
        m_last_line_bearing = line.bearing - line.bearingRange;
        m_range = line.range;
        m_bearingRange = line.bearingRange;
        m_scanWidth = line.scanWidth;
        m_nbins = line.data.size();
    }
    
    if (line.bearingRange > 1600)
    {
        error() << "Cannot deal with arcs larger than a quarter circle. Do you really need the range to be this coarse?";
        return false;
    }

    int from = line.bearing - line.bearingRange/2;
    int to = from + line.bearingRange;

    cv::Mat m = m_img->mat();

    int radius = floor((min(m.rows, m.cols)-1)/2);
    int bincount = line.data.size();

    float bscale = (float)radius/bincount;
    float cx = radius, cy = radius;
    
    int accumulated_delta = mod(line.bearing - m_last_line_bearing, 6400);
    m_last_line_bearing = line.bearing;
    m_image_completed += (float)accumulated_delta / line.scanWidth;

    bool isFullImage = m_image_completed >= 1.0;
    if (isFullImage)
        m_image_completed = 0;

    for (int b = 0; b < bincount; b++) {
        // All calculations assume centre is at (0,0)
        
        float inner_radius = b * bscale;
        float outer_radius = (b+1) * bscale;

        cv::Point2f pt_inner_from(inner_radius*seanet_cached.cos(from), inner_radius*seanet_cached.sin(from));
        cv::Point2f pt_inner_to(inner_radius*seanet_cached.cos(to), inner_radius*seanet_cached.sin(to));
        cv::Point2f pt_outer_from(outer_radius*seanet_cached.cos(from), outer_radius*seanet_cached.sin(from));
        cv::Point2f pt_outer_to(outer_radius*seanet_cached.cos(to), outer_radius*seanet_cached.sin(to));
        
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

    return isFullImage;
}

bool SonarAccumulator::setWholeImage(PolarImage const& image){
    std::vector<int32_t> const& bearing_bins = image.bearing_bins;
    if(!bearing_bins.size()){
        error() << "no bearings: wtf?";
        return false;
    }
    reset();
    const uint32_t num_lines = std::floor(0.5f + (image.rangeEnd - image.rangeStart) / image.rangeConversion);
    const uint32_t start_line = std::floor(0.5f + image.rangeStart / image.rangeConversion);
    const uint32_t end_line = std::floor(0.5f + image.rangeEnd / image.rangeConversion);
    cv::Mat m = m_img->mat();
    const int32_t radius = floor((min(m.rows, m.cols)-1)/2);
    const uint32_t num_bearings = bearing_bins.size()-1;
    const float cx = radius;
    const float cy = radius;
    const float bscale = radius * image.rangeConversion / image.rangeEnd;

    debug(2) << "radius =" << radius << "rows=" << m.rows << "cols=" << m.cols << "rangeEnd=" << image.rangeEnd;

    gem_cached.ensureTablesFor(bearing_bins);

    for(uint32_t line = 0; line < num_lines; line++){
        uint32_t range_line = start_line + line;
        float inner_radius = range_line * bscale;
        float outer_radius = (range_line+1) * bscale;

        assert(range_line != end_line);

        for(uint32_t i = 0; i < num_bearings; i++){
            /*int32_t from = bearing_bins[i];
            int32_t to   = bearing_bins[i+1];

            cv::Point2f pt_inner_from(inner_radius*gem_cos_safe(from), inner_radius*gem_sin_safe(from));
            cv::Point2f pt_inner_to(inner_radius*gem_cos_safe(to), inner_radius*gem_sin_safe(to));
            cv::Point2f pt_outer_from(outer_radius*gem_cos_safe(from), outer_radius*gem_sin_safe(from));
            cv::Point2f pt_outer_to(outer_radius*gem_cos_safe(to), outer_radius*gem_sin_safe(to));
            */
            cv::Point2f pt_inner_from(inner_radius*gem_cached.cos_idx(i), inner_radius*gem_cached.sin_idx(i));
            cv::Point2f pt_inner_to(inner_radius*gem_cached.cos_idx(i+1), inner_radius*gem_cached.sin_idx(i+1));
            cv::Point2f pt_outer_from(outer_radius*gem_cached.cos_idx(i), outer_radius*gem_cached.sin_idx(i));
            cv::Point2f pt_outer_to(outer_radius*gem_cached.cos_idx(i+1), outer_radius*gem_cached.sin_idx(i+1));

            cv::Rect innerbb = arcBound(inner_radius, pt_inner_to, pt_inner_from);
            cv::Rect outerbb = arcBound(outer_radius, pt_outer_to, pt_outer_from);
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
                    //  |  /   it's cw of from and ccw of to.
                    //  | /
                    //  |/
                    bool ccwFrom = ccw<float>(pt_outer_from.x, pt_outer_from.y, x, y);
                    bool ccwTo = ccw<float>(pt_outer_to.x, pt_outer_to.y, x, y);

                    if (ccwFrom || !ccwTo)
                        continue;

                    // If we've got to here, then this must be valid. Shift to the centre and set the value.
                    *pm = image.data[line * num_bearings + i];
                }
            }
        }
    }
    return true;
}

bool SonarAccumulator::setWholeImage(NonUniformPolarMat image){
    if(!image.bearings || !image.ranges){
        error() << "invalid non-uniform polar image: no data";
        return false;
    }
    reset();
    const uint32_t num_lines = image.ranges->size();
    const uint32_t num_bearings = image.bearings->size();
    if(num_lines < 2 || !num_bearings){
        error() << "invalid non-uniform polar image: empty data";
        return false;
    }
    cv::Mat m = m_img->mat();
    const int32_t radius = floor((min(m.rows, m.cols)-1)/2);
    const float cx = radius;
    const float cy = radius;
    const float max_radius_m = image.ranges->back() + (image.ranges->back() - image.ranges->at(num_lines-2))/2;
    const float rscale = radius / max_radius_m;

    // convert centre-of-bin values to edge-of-bin values, and set-up the
    // sin-cos cache: if this proves a bottleneck we should avoid calculating
    // bearing_bins if the cache is already up to data (cache hangs of the
    // number of bins and the first angle)
    {
        std::vector<float> bearing_bins;
        bearing_bins.reserve(num_bearings+1);

        std::vector<float>::const_iterator it, next;
        it = image.bearings->begin();
        next = it;
        bearing_bins.push_back(*it - (*next-*it)/2);
        next++;
        for(;next != image.bearings->end(); next++, it++)
            bearing_bins.push_back((*it + *next)/2);
        bearing_bins.push_back(*it + (*it - bearing_bins.back()));

        cached_trig.ensureTablesFor(bearing_bins);
    }

    float inner_radius;
    float outer_radius;
    for(uint32_t line = 0; line < num_lines; line++){
        // !!! FIXME: ranges should be in image coordinates, not m
        if(line == 0){
            inner_radius = rscale * ((*image.ranges)[line] - ((*image.ranges)[line+1]-(*image.ranges)[line])/2);
            outer_radius = rscale * ((*image.ranges)[line] + (*image.ranges)[line+1])/2;
        }else if(line == num_lines-1){
            inner_radius = rscale * ((*image.ranges)[line] + (*image.ranges)[line-1])/2;
            outer_radius = radius;
        }else{
            inner_radius = rscale * ((*image.ranges)[line] + (*image.ranges)[line-1])/2;
            outer_radius = rscale * ((*image.ranges)[line] + (*image.ranges)[line+1])/2;
        }

        for(uint32_t i = 0; i < num_bearings; i++){
            cv::Point2f pt_inner_from(inner_radius*cached_trig.cos_idx(i), inner_radius*cached_trig.sin_idx(i));
            cv::Point2f pt_inner_to(inner_radius*cached_trig.cos_idx(i+1), inner_radius*cached_trig.sin_idx(i+1));
            cv::Point2f pt_outer_from(outer_radius*cached_trig.cos_idx(i), outer_radius*cached_trig.sin_idx(i));
            cv::Point2f pt_outer_to(outer_radius*cached_trig.cos_idx(i+1), outer_radius*cached_trig.sin_idx(i+1));

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
                    //  |  /   it's cw of from and ccw of to.
                    //  | /
                    //  |/
                    bool ccwFrom = ccw<float>(pt_outer_from.x, pt_outer_from.y, x, y);
                    bool ccwTo = ccw<float>(pt_outer_to.x, pt_outer_to.y, x, y);

                    if (ccwFrom || !ccwTo)
                        continue;

                    // If we've got to here, then this must be valid. Shift to the centre and set the value.
                    *pm = image.mat.at<uint8_t>(line, i);
                }
            }
        }
    }
    return true;
}

void SonarAccumulator::setSize(uint32_t size){
    if(m_size != size){
        m_size = size;
        reset();
    }
}

boost::shared_ptr<Image> SonarAccumulator::img() const
{
    return m_img;
}

cv::Mat SonarAccumulator::mat() const
{
    return m_img->mat();
}


