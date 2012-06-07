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

#include "dataSeries.h"

#include <boost/make_shared.hpp>

#include <utility/foreach.h>
#include <utility/math.h>
#include <utility/rounding.h>

#include "dataWindow.h"

namespace w = liquid::water;
namespace wi = liquid::water::internal;

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

