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
#include "internal/dataSeries.h"

// !!!FIXME bad dependency, necessary because we don't send time on the wire
// with telemetry data yet
#include <utility/time.h>


namespace w = liquid::water;


w::SeriesConfig w::Radians_Angle_Graph = {
    0.0f, M_PI - M_PI/8, M_PI/8, M_PI, GraphSeriesMode::Modulus
};

w::SeriesConfig w::Degrees_Angle_Graph = {
    0.0f, 315.0f, 45.0f, 360.0f, GraphSeriesMode::Modulus
};

// "unlimited", but the maximum vertical scale will be limited to 0--100, so if
// the data goes above 100 it'll go off the top.
w::SeriesConfig w::Percent_Graph = {
    0.0f, 0.0f, 100.0f, 100.0f, GraphSeriesMode::Unlimited
};

w::SeriesConfig w::Unlimited_Graph = {
    -std::numeric_limits<float>::max(), 0.0f, 
    0.0f, std::numeric_limits<float>::max(),
    GraphSeriesMode::Unlimited
};




// - DataSeries

w::DataSeries::DataSeries(SeriesConfig const& config, QString series_name)
    : m(new internal::DataSeries(config, series_name)){
}

void w::DataSeries::postData(double const& value, double const& time){
    m->postData(value, time);
}

void w::DataSeries::postData(QVariant const& v){
    postData(v, cauv::nowDouble());
}

void w::DataSeries::postData(QVariant const& value, double const& time){
    bool ok = false;
    const double val = value.toDouble(&ok);
    if(ok)
        postData(val, time);
}

