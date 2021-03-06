/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_WATER_INTERNAL_DATA_WINDOW_H__
#define __LIQUID_WATER_INTERNAL_DATA_WINDOW_H__

#include <deque>

#include <boost/noncopyable.hpp>

#include <QPolygonF>
#include <QPointF>

#include "graph.h"

namespace liquid{
namespace water{
namespace internal{

class DataWindow: public boost::noncopyable{
    public:
        DataWindow();
        
        /* config is used to wrap / clamp samples into range as appropriate */
        void addSample(Map::MinAvgMax const& mam, double const& time, SeriesConfig const& config);

        void removeSamplesBefore(double const& t);
        
        float const& min() const{ return m_min; }
        float const& max() const{ return m_max; }

        double const& minTime() const{ return m_sample_times.front(); }
        double const& maxTime() const{ return m_sample_times.back(); }

        QPolygonF regionAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const;
        QPolygonF valuesAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const;

        std::size_t size() const;

    private:
        void _recalcMinMax();

        std::deque<double> m_sample_times;
        std::deque<float>  m_min_values;
        std::deque<float>  m_mean_values;
        std::deque<float>  m_max_values;
        float m_min;
        float m_max;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_INTERNAL_DATA_WINDOW_H__
