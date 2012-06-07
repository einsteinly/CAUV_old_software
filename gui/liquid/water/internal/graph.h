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

#include <list>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>

#include <QGraphicsItem>

#include "../graph.h"
#include "../dataSeries.h"
#include "persistentMap.h"

namespace liquid{
namespace water{
namespace internal{

class DataWindow;
class DataSeries;
class Graph;
class GraphAxes;
class GraphLegend;

typedef boost::shared_ptr<DataWindow> DataWindow_ptr;

/*
 * Each time a graph is drawn:
 *
 *  First the graph determines the size at which it will be rendered, which
 *  defines the time-resolution necessary for the plot.
 *
 *  Then the graph collects a DataWindow from each of its component series
 *  representing the data from that series for the time period that will be
 *  rendered, sampling at the resolution that will be drawn.
 *
 *  Next the graph calculates the y-axis scaling by querying min() and max() on
 *  each data window.
 *
 *  Finally, the graph draws each dataseries, also rendering the minimum and
 *  maximum lines if they differ significantly from the mean value
 *
 */

/*class RedrawManager{
    public:
        RedrawManager(QGraphicsScene* scene)
            : m_scene(scene){
        }

        void add(Graph* g){
        }

        void remove(Graph* g){
        }

        void setDirty(Graph* g){
        }

        void setClean(Graph* c){
        }
    
    private:
        QGraphicsScene* m_scene;
        float m_max_updates_per_second;

        std::map<Graph*,bool?float?
};*/

namespace Graph_Const {
const static float Max_Resolution = 1200; // max data-points per data series, at any one time
} // namespace Graph_Const

class Graph: public boost::noncopyable{
    public:
        // public types
        struct SeriesData{
            DataSeries_ptr data_series;
            DataWindow_ptr data_window;
        };
    
    public:
        Graph(GraphConfig const& config, QString name, QGraphicsItem* owner);
        
        void addDataSeries(DataSeries_ptr data_series);
        void removeDataSeries(DataSeries_ptr data_series);
        void updateDataWindows(double const& tstart, double const& tend, uint32_t resolution);

        // Delegated QGraphicsItem Implementation
        QRectF boundingRect() const;
        void requiresUpdate() const;
        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *option,
                   QWidget *widget);


         // Other drawing-associated Implementation
         void setRect(QRectF const& rect);
    
    private:
        void _discardAndUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution);
        void _incrementalUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution);
        void _updateDataMinMax();
        void _recalcMinMaxMaxMin();

        static bool hasInvalidWindow(SeriesData const& d);


        QGraphicsItem* m_owner;
        GraphConfig m_config;
        std::list<SeriesData> m_data_series;
        float m_data_min; // used for vertical scale determination, these
                          // values are affected by the min & max values in the
                          // series config
        float m_data_max; // "
        
        // State for drawing
        QRectF m_rect;
        bool m_rect_changed;
        float m_maximum_minimum;
        float m_minimum_maximum;
        
        // children
        GraphAxes* m_axes;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_GRAPH_IMPL_H__
