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

#ifndef __LIQUID_WATER_GRAPH_IMPL_H__
#define __LIQUID_WATER_GRAPH_IMPL_H__

#include "../graph.h"
#include "persistentMap.h"

namespace liquid{
namespace water{
namespace internal{

class Graph{
    public:
        Graph(GraphConfig const& config, QString name)
            : m_config(config),
              m_data(name){
        }
        
        void postData(double const& value, double const& time){
            m_data.insert(time, value);
        }
        
        

        GraphConfig m_config;
        Map m_data;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_GRAPH_IMPL_H__
