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

#include <boost/shared_ptr.hpp>

#include <QGraphicsItem>

namespace liquid{
namespace water{

namespace GraphSeriesMode{
enum e{Unlimited, Wraps_At_Max_Max};
}

struct GraphConfig{
    float time_period; // only a hint, in seconds
};

struct SeriesConfig{
    // control vertical scaling
    float minimum_minimum;
    float maximum_minimum;
    float minimum_maximum;
    float maximum_maximum;
    GraphSeriesMode::e mode;
};

static GraphConfig One_Minute = {
    60.0f
};

static SeriesConfig Radians_Angle_Graph = {
    0.0f, M_PI - M_PI/8, M_PI/8, M_PI, GraphSeriesMode::Wraps_At_Max_Max
};

static SeriesConfig Degrees_Angle_Graph = {
    0.0f, 315.0f, 45.0f, 360.0f, GraphSeriesMode::Wraps_At_Max_Max
};

// "unlimited", but the maximum vertical scale will be limited to 0--100, so if
// the data goes above 100 it'll go off the top.
static SeriesConfig Percent_Graph = {
    0.0f, 0.0f, 100.0f, 100.0f, GraphSeriesMode::Unlimited
};

// implementation in internal classes
namespace internal{
class Graph;
class DataSeries;
} // namespace internal

class Graph;
class DataSeries;

typedef boost::shared_ptr<DataSeries> DataSeries_ptr;

// Represents one data series. Create Graphs by adding these to a Graph object
class DataSeries{
    public:
        DataSeries(SeriesConfig const& config, QString series_name);

        /* Add a data value to the data series.
         *
         *  value: the value, easy
         *  time: time in floating point seconds. Should correspond to the
         *        values returned by the nowDouble() function in utility/time.h
         */
        void postData(double const& value, double const& time);
    
    private:
        // internal::Graph can call methods directly on m
        friend class internal::Graph;

        boost::shared_ptr<internal::DataSeries> m;
};

// Draws zero or more data series.
class Graph: public QGraphicsItem{
    public:
        Graph(GraphConfig const& config, QString name, QGraphicsItem* parent=0);

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

