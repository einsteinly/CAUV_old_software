/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "dataWindow.h"

#include <utility/foreach.h>
#include <utility/math.h>
#include <utility/rounding.h>

namespace w = liquid::water;
namespace wi = liquid::water::internal;

wi::DataWindow::DataWindow()
    : m_sample_times(),
      m_min_values(),
      m_mean_values(),
      m_max_values(),
      m_min( std::numeric_limits<float>::max()),
      m_max(-std::numeric_limits<float>::max()){
}

/* config is used to wrap / clamp samples into range as appropriate */
void wi::DataWindow::addSample(Map::MinAvgMax const& mam, double const& time, SeriesConfig const& config){
    float min;
    float avg;
    float max;
    if(m_sample_times.size() && config.mode == GraphSeriesMode::Modulus){
        const double base = config.maximum_maximum - config.minimum_minimum;
        /*float const& last = m_mean_values.back();
        const float zero = base * std::floor(last/base);
        
        min = zero + cauv::angleMod(mam.min, base);
        avg = zero + cauv::angleMod(mam.average, base);
        max = zero + cauv::angleMod(mam.max, base);*/
        min = cauv::mod(mam.min, base);
        avg = cauv::mod(mam.average, base);
        max = cauv::mod(mam.max, base);

        //debug() << min << avg << max;

    }else if(config.mode == GraphSeriesMode::Clamp){
        min = clamp(config.minimum_minimum, mam.min, config.maximum_maximum);
        avg = clamp(config.minimum_minimum, mam.average, config.maximum_maximum);
        max = clamp(config.minimum_minimum, mam.max, config.maximum_maximum);
    }else{
        min = mam.min;
        avg = mam.average;
        max = mam.max;
    }
    m_sample_times.push_back(time);
    m_min_values.push_back(min);
    m_mean_values.push_back(avg);
    m_max_values.push_back(max);
    if(mam.min < m_min)
        m_min = mam.min;
    if(mam.max > m_max)
        m_max = mam.max;
}

void wi::DataWindow::removeSamplesBefore(double const& t){
    bool recalcminmax = false;
    while((m_sample_times.size() > 1 && m_sample_times[1] <= t) ||
          (m_sample_times.size() == 1 && m_sample_times.front() < t)){
        if(m_min_values.front() == m_min)
            recalcminmax = true;
        if(m_max_values.front() == m_max)
            recalcminmax = true;
        m_sample_times.pop_front();
        m_min_values.pop_front();
        m_mean_values.pop_front();
        m_max_values.pop_front();
    }
    if(recalcminmax)
        _recalcMinMax();
}

QPolygonF wi::DataWindow::regionAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const{
    QPolygonF r;
    const int n = m_sample_times.size();
    r.reserve(2*n);
    
    int start = 0;
    int end = n;
    int i;

    for(i = 0; i < end; i++){
        if(std::fabs(m_max_values[i] - m_mean_values[i]) > 2*yscale ||
           std::fabs(m_min_values[i] - m_mean_values[i]) > 2*yscale){
            start = i;
            break;
        }
    }
    if(i == end)
        start = end;

    for(i = n-1; i > start+1; i--){
        if(std::fabs(m_max_values[i] - m_mean_values[i]) > 2*yscale ||
           std::fabs(m_min_values[i] - m_mean_values[i]) > 2*yscale){
            end = i+1;
            break;
        }
    }

    for(i = start; i < end; i++){
        r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                            (ymax - m_max_values[i]) * yscale + origin.y()));
    }

    for(i = end-1; i >= start; i--){
        r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                            (ymax - m_min_values[i]) * yscale + origin.y()));
    }

    return r;
}

QPolygonF wi::DataWindow::valuesAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const{
    QPolygonF r;
    const int n = m_sample_times.size();
    r.reserve(n);
    for(int i = 0; i < n; i++){
        r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                            (ymax - m_mean_values[i]) * yscale + origin.y()));
    }
    return r;
}

std::size_t wi::DataWindow::size() const{
    return m_sample_times.size();
}

void wi::DataWindow::_recalcMinMax(){
    if(m_sample_times.size()){
        m_min = std::numeric_limits<float>::max();
        m_max = -std::numeric_limits<float>::max();
    }
    foreach(float const& f, m_min_values){
        if(f < m_min)
            m_min = f;
    }
    foreach(float const& f, m_max_values){
        if(f > m_max)
            m_max = f;
    }
}

