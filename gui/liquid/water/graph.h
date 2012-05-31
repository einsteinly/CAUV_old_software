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

#include <boost/scoped_ptr.hpp>

#include <QGraphicsItem>

namespace liquid{
namespace water{

namespace GraphMode{
enum e{Unlimited, Clamps, Wraps };
}
struct GraphConfig{
    float time_period; // only a hint, in seconds
    float minimum;
    float maximum;
    GraphMode::e mode;
};

GraphConfig Radians_Angle_Graph = {
    60.0f, 0.0f, 3.14159265359f, GraphMode::Wraps
};

GraphConfig Degrees_Angle_Graph = {
    60.0f, 0.0f, 360.0f, GraphMode::Wraps
};

GraphConfig Percent_Graph = {
    60.0f, 0.0f, 100.0f, GraphMode::Clamps
};

// implementation in a private class
namespace internal{
class Graph;
} // namespace internal

class Graph: public QGraphicsItem{
    public:
        Graph(GraphConfig const& config, QString name);

        void postData(double const& value, double const& time);

        // QGraphicsItem implementation
        QRectF boundingRect() const;
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    private:
        boost::scoped_ptr<internal::Graph> m;
};

} // namespace water
} // namespace liquid


#endif // ndef __LIQUID_WATER_GRAPH_H__

