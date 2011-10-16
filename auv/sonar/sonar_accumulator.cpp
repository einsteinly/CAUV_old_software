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

float cauv::msgPolarAngleToRadians(int32_t bearing){
    return M_PI * bearing / (3200.0*0x10000);
}

SonarAccumulator::SonarAccumulator(uint32_t size)
{   
    m_size = size;
    reset();
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

static std::vector<float> gem_sins;
static std::vector<float> gem_coss;
static int32_t gem_sins_start_from = 0;

static float gem_sin_nocache(int32_t bearing){
    return std::sin(M_PI * bearing / (3200.0*0x10000));
}

static float gem_cos_nocache(int32_t bearing){
    return gem_sin_nocache(bearing + 1600*0x10000);
}

void ensureGemAngleTablesFor(std::vector<int32_t> angles){
    if(angles[0] == gem_sins_start_from &&
       angles.size() == gem_sins.size()){
        return;
    }
    gem_sins.clear();
    gem_coss.clear();
    gem_sins.reserve(angles.size());
    gem_coss.reserve(angles.size());
    for(uint32_t i = 0; i < angles.size(); i++){
        gem_sins[i] = gem_sin_nocache(angles[i]);
        gem_coss[i] = gem_cos_nocache(angles[i]);
    }
}

static float gem_sin_safe(int32_t bearing){
    // !!! TODO: better caching scheme....
    static std::map<int32_t, float> sin_cache;

    std::map<int32_t, float>::const_iterator i = sin_cache.find(bearing);
    if(i == sin_cache.end()){
        const float r = gem_sin_nocache(bearing);
        sin_cache[bearing] = r;
        if(sin_cache.size() > 100000){
            std::map<int32_t, float>::iterator to_remove = sin_cache.lower_bound(rand() % 6400*0x10000);
            if(to_remove == sin_cache.end()){
                // !!! really not good: values probably lie outside 0--6400*0x10000
                sin_cache.erase(sin_cache.begin());
            }else{
                sin_cache.erase(to_remove);
            }
        }
        return r;
    }else{
        return i->second;
    }
}

static float gem_cos_safe(int32_t bearing){
    return gem_sin_safe(bearing + 1600*0x10000);
}

static float gem_sin_idx(uint32_t bearing_idx){
    return gem_sins[bearing_idx];
}

static float gem_cos_idx(uint32_t bearing_idx){
    return gem_coss[bearing_idx];
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

    return isFullImage;
}

bool SonarAccumulator::setWholeImage(PolarImage const& image){
    std::vector<int32_t> const& bearing_bins = image.bearing_bins;
    if(!bearing_bins.size()){
        error() << "no bearings: wtf?";
        return false;
    }
    reset();
    const uint32_t num_lines = image.rangeEnd - image.rangeStart;
    cv::Mat m = m_img->mat();
    const int32_t radius = floor((min(m.rows, m.cols)-1)/2);
    debug() << "radius =" << radius << "rows=" << m.rows << "cols=" << m.cols << "rangeEnd=" << image.rangeEnd;
    const uint32_t num_bearings = bearing_bins.size()-1;
    const float cx = radius;
    const float cy = radius;
    const float bscale = float(radius)/image.rangeEnd;

    ensureGemAngleTablesFor(bearing_bins);

    for(uint32_t line = 0; line < num_lines; line++){
        uint32_t range_line = image.rangeStart + line;
        float inner_radius = range_line * bscale;
        float outer_radius = (range_line+1) * bscale;

        for(uint32_t i = 0; i < num_bearings; i++){
            /*int32_t from = bearing_bins[i];
            int32_t to   = bearing_bins[i+1];

            cv::Point2f pt_inner_from(inner_radius*gem_cos_safe(from), inner_radius*gem_sin_safe(from));
            cv::Point2f pt_inner_to(inner_radius*gem_cos_safe(to), inner_radius*gem_sin_safe(to));
            cv::Point2f pt_outer_from(outer_radius*gem_cos_safe(from), outer_radius*gem_sin_safe(from));
            cv::Point2f pt_outer_to(outer_radius*gem_cos_safe(to), outer_radius*gem_sin_safe(to));
            */
            cv::Point2f pt_inner_from(inner_radius*gem_cos_idx(i), inner_radius*gem_sin_idx(i));
            cv::Point2f pt_inner_to(inner_radius*gem_cos_idx(i+1), inner_radius*gem_sin_idx(i+1));
            cv::Point2f pt_outer_from(outer_radius*gem_cos_idx(i), outer_radius*gem_sin_idx(i));
            cv::Point2f pt_outer_to(outer_radius*gem_cos_idx(i+1), outer_radius*gem_sin_idx(i+1));

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
                    *pm = image.data[line * num_bearings + i];
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


