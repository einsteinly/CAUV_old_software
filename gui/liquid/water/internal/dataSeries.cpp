/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "dataSeries.h"

#include <boost/make_shared.hpp>

#include <utility/foreach.h>
#include <utility/math.h>
#include <utility/rounding.h>
#include <utility/time.h>

#include "dataWindow.h"

namespace w = liquid::water;
namespace wi = liquid::water::internal;

wi::DataSeries::DataSeries(SeriesConfig const& config, QString series_name)
    : m_data(series_name),
      m_insert_batch(),
      m_insert_batch_size(1),
      m_last_time(cauv::nowDouble()),
      m_last_value(0),
      m_dirty_in_graph(),
      m_config(config){
}

void  wi::DataSeries::postData(double value, double const& time){
    //debug() << "postData" << value << time;
    // ensure that mean calculations with angles work correctly (at the
    // expense of denormalised angles):
    if(m_config.mode == GraphSeriesMode::Modulus){
        const double base = m_config.maximum_maximum - m_config.minimum_minimum;
        const float zero = base * std::floor(m_last_value/base);
        value = zero + cauv::mod(value, base);
    }
    
    // adapt insertion batch size to try to get about two-four updates per
    // second
    double now = cauv::nowDouble();
    if(m_insert_batch_size > 1 &&  now - m_last_time > 2.0){
        m_insert_batch_size = 1;
    }else if(m_insert_batch_size > 1 && now - m_last_time > 0.5){
        m_insert_batch_size--;
    }else if(m_insert_batch_size < 10000 && now - m_last_time < 0.25){
        m_insert_batch_size++;
    }

    m_insert_batch.push_back(std::pair<double,double>(time, value));
    if(m_insert_batch.size() >= m_insert_batch_size){
        //debug() << "insert batch:" << m_insert_batch.size();
        m_data.insertMultiple(m_insert_batch.begin(), m_insert_batch.end());
        m_insert_batch.clear();
        foreach(wi::Graph* g, m_in_graphs)
            g->requiresUpdate();
        m_last_time = now;
        // hit the database only if there's something new
        for(std::map<wi::Graph*, bool>::iterator i = m_dirty_in_graph.begin(); i != m_dirty_in_graph.end(); i++)
            i->second = true;
    }

    m_last_value = value;
}

void  wi::DataSeries::addGraph(wi::Graph* g){
    m_in_graphs.push_back(g);
}

wi::DataWindow_ptr  wi::DataSeries::snapshot(double const& tstart, double const& tend, unsigned resolution, wi::Graph* for_graph){ 
    DataWindow_ptr r = boost::make_shared<DataWindow>();
    
    // if graph wants all the data, can't optimise about not hitting the
    // database unless there's anything new (so ignore m_dirty)
    //if(!m_dirty_in_graph[for_graph])
    //    return;
    //m_dirty_in_graph[for_graph] = false;

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

void  wi::DataSeries::updateSnapshot(DataWindow_ptr w, double const& tstart, double const& tend, uint32_t resolution, wi::Graph* for_graph){
    w->removeSamplesBefore(tstart);

    const double dt = (tend - tstart) / resolution;
    if(!w->size()){
        w = snapshot(tstart, tend, resolution, for_graph);
        return;
    }
    
    if(!m_dirty_in_graph[for_graph])
        return;
    m_dirty_in_graph[for_graph] = false;

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

