/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_WATER_INTERNAL_DATA_SERIES_H__
#define __LIQUID_WATER_INTERNAL_DATA_SERIES_H__

#include <vector>
#include <utility>

#include <boost/noncopyable.hpp>

#include "graph.h"
#include "persistentMap.h"

namespace liquid{
namespace water{
namespace internal{

class DataSeries: public boost::noncopyable{
    public:
        DataSeries(SeriesConfig const& config, QString series_name);

        void postData(double value, double const& time);

        void addGraph(internal::Graph* g);
        
        DataWindow_ptr snapshot(double const& tstart, double const& tend, unsigned resolution, Graph* for_graph);

        void updateSnapshot(DataWindow_ptr w, double const& tstart, double const& tend, uint32_t resolution, Graph* for_graph);

        SeriesConfig const& config() const;

    private:
        Map m_data;
        
        std::vector< std::pair<double, double> > m_insert_batch;
        unsigned m_insert_batch_size;
        double   m_last_time;
        double   m_last_value;
        std::map<Graph*, bool> m_dirty_in_graph;

        SeriesConfig m_config;

        std::vector<internal::Graph*> m_in_graphs;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_INTERNAL_DATA_SERIES_H__
