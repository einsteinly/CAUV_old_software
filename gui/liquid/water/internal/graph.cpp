/* Copyright 2012 Cambridge Hydronautics Ltd.
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


#include "graph.h"

namespace w = liquid::water;
namespace wi = liquid::water::internal;



// - DataWindow

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
    for(int i = 0; i < n; i++){
        r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                            (ymax - m_max_values[i]) * yscale + origin.y()));
    }

    for(int i = n-1; i >= 0; i--){
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


// - DataSeries

wi::DataSeries::DataSeries(SeriesConfig const& config, QString series_name)
    : m_data(series_name),
      m_insert_batch(),
      m_last_value(0),
      m_config(config){
}

void  wi::DataSeries::postData(double value, double const& time){
    // ensure that mean calculations with angles work correctly (at the
    // expense of denormalised angles):
    if(m_config.mode == GraphSeriesMode::Modulus){
        const double base = m_config.maximum_maximum - m_config.minimum_minimum;
        const float zero = base * std::floor(m_last_value/base);
        value = zero + cauv::mod(value, base);
    }

    m_insert_batch.push_back(std::pair<double,double>(time, value));
    if(m_insert_batch.size() >= Data_Batch_Size){
        //debug() << "insert batch:" << m_insert_batch.size();
        m_data.insertMultiple(m_insert_batch.begin(), m_insert_batch.end());
        m_insert_batch.clear();
        foreach(wi::Graph* g, m_in_graphs)
            g->requiresUpdate();
    }

    m_last_value = value;
}

void  wi::DataSeries::addGraph(wi::Graph* g){
    m_in_graphs.push_back(g);
}

wi::DataWindow_ptr  wi::DataSeries::snapshot(double const& tstart, double const& tend, unsigned resolution){
    DataWindow_ptr r = boost::make_shared<DataWindow>();
    if(resolution == 0)
        resolution++;
    const double dt = (tend-tstart) / resolution;
    Map::MinAvgMax x = {0,0,0};
    for(double t = tstart; t <= tend-dt/2; t += dt)
        if(m_data.minAvgMaxOfRange(t, t+dt, x))
            r->addSample(x, t+dt/2, m_config);
    //debug() << "snapshot" << tstart << tend << resolution << "->" << r->size() << "values";
    // !!! TODO (important) if there were no values in the snapshot, DO
    // NOT TRY AGAIN SOON: back off (this might mean reducing the
    // frame-rate, for example), because getting the whole snapshot can
    // be an expensive operation, that crowds out the actual insertion
    // of data, so we get stuck in a loop getting expensive empty
    // snapshots
    return r;
}

void  wi::DataSeries::updateSnapshot(DataWindow_ptr w, double const& tstart, double const& tend, uint32_t resolution){
    w->removeSamplesBefore(tstart);
    const double dt = (tend - tstart) / resolution;
    if(!w->size()){
        w = snapshot(tstart, tend, resolution);
        return;
    }
    Map::MinAvgMax x = {0, 0, 0};
    for(double t = w->maxTime(); t < tend-dt/2; t += dt)
        if(m_data.minAvgMaxOfRange(t, t+dt, x))
            w->addSample(x, t+dt/2, m_config);
    //debug() << "incremental snapshot" << tstart << tend << resolution
    //        << "->" << (tend - w->maxTime()) / dt << "values";
}

w::SeriesConfig const&  wi::DataSeries::config() const{
    return m_config;
}


// - Free Functions
bool wi::hasInvalidWindow(SeriesData const& d){
    return !d.data_window || !d.data_window->size();
}


