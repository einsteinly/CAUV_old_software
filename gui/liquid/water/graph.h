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

#ifndef __LIQUID_WATER_GRAPH_H__
#define __LIQUID_WATER_GRAPH_H__

#include <cmath>
#include <limits>

#include <boost/shared_ptr.hpp>

#include <QGraphicsItem>

namespace liquid{
namespace water{

struct GraphConfig{
    float time_period; // only a hint, in seconds
};

extern GraphConfig One_Minute;


// implementation in internal classes
namespace internal{
class Graph;
class DataSeries;
} // namespace internal

class Graph;
class DataSeries;

typedef boost::shared_ptr<DataSeries> DataSeries_ptr;

// Draws zero or more data series.
class Graph: public QGraphicsItem{
    public:
        Graph(GraphConfig const& config, QGraphicsItem* parent=0);

        void addDataSeries(DataSeries_ptr data_series);

        void setRect(QRectF const& r);

        // QGraphicsItem implementation
        QRectF boundingRect() const;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    private:
        boost::shared_ptr<internal::Graph> m;
};

} // namespace water
} // namespace liquid


#endif // ndef __LIQUID_WATER_GRAPH_H__

