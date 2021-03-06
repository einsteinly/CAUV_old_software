/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_WATER_DATA_SERIES_H__
#define __LIQUID_WATER_DATA_SERIES_H__

#include <cmath>
#include <limits>

#include <QString>
#include <QVariant>

#include <boost/shared_ptr.hpp>

namespace liquid{
namespace water{

namespace GraphSeriesMode{
enum e{
    Unlimited,
    Clamp,
    Modulus
};
}

struct SeriesConfig{
    // control vertical scaling
    // for angle graphs, maximum_maximum - minimum_minimum is used as the
    // modulus for angle plotting magic
    float minimum_minimum;
    float maximum_minimum;
    float minimum_maximum;
    float maximum_maximum;
    GraphSeriesMode::e mode;
};

// standard graph configs:
extern SeriesConfig Radians_Angle_Graph;
extern SeriesConfig Degrees_Angle_Graph;
extern SeriesConfig Percent_Graph;
extern SeriesConfig Unlimited_Graph;


// implementation in internal classes
namespace internal{
class Graph;
class DataSeries;
} // namespace internal

// Represents one data series. Create Graphs by adding these to a Graph object
class DataSeries: public QObject{
    Q_OBJECT

    public:
        DataSeries(SeriesConfig const& config, QString series_name);

        /* Add a data value to the data series.
         *
         *  value: the value, easy
         *  time: time in floating point seconds. Should correspond to the
         *        values returned by the nowDouble() function in utility/time.h
         */
        void postData(double const& value, double const& time);
    
    public Q_SLOTS:
        void postData(QVariant const& v);
        void postData(QVariant const& v, double const& time);
    
    private:
        // internal::Graph can call methods directly on m
        friend class internal::Graph;

        boost::shared_ptr<internal::DataSeries> m;
};


} // namespace water
} // namespace liquid


#endif // ndef __LIQUID_WATER_DATA_SERIES_H__

