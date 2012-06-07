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
        
        DataWindow_ptr snapshot(double const& tstart, double const& tend, unsigned resolution);

        void updateSnapshot(DataWindow_ptr w, double const& tstart, double const& tend, uint32_t resolution);

        SeriesConfig const& config() const;

    private:
        Map m_data;
        enum{Data_Batch_Size = 20};
        std::vector< std::pair<double, double> > m_insert_batch;
        double m_last_value;

        SeriesConfig m_config;

        std::vector<internal::Graph*> m_in_graphs;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_INTERNAL_DATA_SERIES_H__
