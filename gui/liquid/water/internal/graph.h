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
#include <deque>
#include <algorithm>

#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QPolygonF>
#include <QPen>
#include <QBrush>

#include <debug/cauv_debug.h>

#include <utility/time.h>
#include <utility/foreach.h>

#include "../graph.h"
#include "persistentMap.h"

namespace liquid{
namespace water{
namespace internal{

class DataWindow;
class DataSeries;
class Graph;

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

class DataWindow: public boost::noncopyable{
    public:
        DataWindow()
            : m_sample_times(),
              m_min_values(),
              m_mean_values(),
              m_max_values(),
              m_min(-std::numeric_limits<float>::max()),
              m_max( std::numeric_limits<float>::max()){
        }

        void addSample(Map::MinAvgMax const& mam, double const& time){
            m_sample_times.push_back(time);
            m_min_values.push_back(mam.min);
            m_mean_values.push_back(mam.average);
            m_max_values.push_back(mam.max);
            if(mam.min < m_min)
                m_min = mam.min;
            if(mam.max > m_max)
                m_max = mam.max;
        }

        void removeSamplesBefore(double const& t){
            while(m_sample_times.front() < t && m_sample_times.size()){
                m_sample_times.pop_front();
                m_min_values.pop_front();
                m_mean_values.pop_front();
                m_max_values.pop_front();
            }
        }
        
        float const& min() const{ return m_min; }
        float const& max() const{ return m_max; }

        double const& minTime() const{ return m_sample_times.front(); }
        double const& maxTime() const{ return m_sample_times.back(); }

        QPolygonF regionAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const{
            QPolygonF r;
            const int n = m_sample_times.size();
            r.reserve(2*n);
            for(int i = 0; i < n; i++){
                r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                                    (ymax - m_max_values[i]) * yscale + origin.y()));
            }

            for(int i = n-1; i >= 0; i--){
                r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                                    (ymax - m_min_values[i]) * yscale + origin.y()));
            }

            return r;
        }

        QPolygonF valuesAtScale(QPointF origin, double const& tmin, float ymax, float xscale, float yscale) const{
            QPolygonF r;
            const int n = m_sample_times.size();
            r.reserve(n);
            for(int i = 0; i < n; i++){
                r.push_back(QPointF((m_sample_times[i] - tmin) * xscale + origin.x(),
                                    (ymax - m_mean_values[i]) * yscale + origin.y()));
            }
            return r;
        }

        std::size_t size() const{
            return m_sample_times.size();
        }

    private:
        std::deque<double> m_sample_times;
        std::deque<float>  m_min_values;
        std::deque<float>  m_mean_values;
        std::deque<float>  m_max_values;
        float m_min;
        float m_max;
};

class DataSeries: public boost::noncopyable{
    public:
        DataSeries(SeriesConfig const& config, QString series_name)
            : m_data(series_name),
              m_insert_batch(),
              m_config(config){
        }

        void postData(double const& value, double const& time){
            //m_data.insert(time, value);

            m_insert_batch.push_back(std::pair<double,double>(time, value));
            if(m_insert_batch.size() == Data_Batch_Size){
                m_data.insertMultiple(m_insert_batch.begin(), m_insert_batch.end());
                m_insert_batch.clear();
            }
        }
        
        DataWindow_ptr snapshot(double const& tstart, double const& tend, unsigned resolution){
            DataWindow_ptr r = boost::make_shared<DataWindow>();
            if(resolution == 0)
                resolution++;
            const double dt = (tend-tstart) / resolution;
            Map::MinAvgMax x = {0};            
            for(double t = tstart; t <= tend-dt/2; t += dt)
                if(m_data.minAvgMaxOfRange(t, t+dt, x))
                    r->addSample(x, t+dt/2);
            //debug() << "snapshot" << tstart << tend << resolution << "->" << r->size() << "values";
            return r;
        }

        void updateSnapshot(DataWindow_ptr w, double const& tstart, double const& tend, uint32_t resolution){
            w->removeSamplesBefore(tstart);
            const double dt = (tend - tstart) / resolution;
            if(!w->size()){
                w = snapshot(tstart, tend, resolution);
                return;
            }
            Map::MinAvgMax x = {0};            
            for(double t = w->maxTime(); t < tend-dt/2; t += dt)
                if(m_data.minAvgMaxOfRange(t, t+dt, x))
                    w->addSample(x, t+dt/2);
            //debug() << "incremental snapshot" << tstart << tend << resolution
            //        << "->" << (tend - w->maxTime()) / dt << "values";
        }

        SeriesConfig const& config() const{
            return m_config;
        }

    private:
        Map m_data;
        enum{Data_Batch_Size = 40};
        std::vector< std::pair<double, double> > m_insert_batch;

        SeriesConfig m_config;
};

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

class GraphLegend: public QGraphicsItem{
};

class GraphAxes: public QGraphicsItem{
    public:
        GraphAxes(QRectF const& rect, QGraphicsItem* parent)
            : QGraphicsItem(parent),
              m_rect(rect){
        }

        QRectF contentsRect() const{
            return m_rect;
        }

        void setRect(QRectF const& rect){
            prepareGeometryChange();
            m_rect = rect;
        }

        // QGraphicsItem Implementation:
        QRectF boundingRect() const{
            return m_rect;
        }

        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *option,
                   QWidget *widget){
        }
    
    private:
        QRectF m_rect;
};

namespace Graph_Const {
const static float Max_Resolution = 1200; // max data-points per data series, at any one time
} // namespace Graph_Const


// !!! tidy up where these little bits go
struct SeriesData{ DataSeries_ptr data_series; DataWindow_ptr data_window; };
bool hasInvalidWindow(SeriesData const& d){
    return !d.data_window;
}

class Graph: public boost::noncopyable{
    public:
        Graph(GraphConfig const& config, QString name, QGraphicsItem* owner)
            : boost::noncopyable(),
              m_owner(owner),
              m_config(config),
              m_data_series(),
              m_data_min(0),
              m_data_max(0),
              m_rect(0,0,200,100),
              m_rect_changed(true),
              m_axes(new GraphAxes(m_rect, owner)){
        }
        
        void addDataSeries(DataSeries_ptr data_series){
            SeriesData d = {data_series, DataWindow_ptr()};
            m_data_series.insert(m_data_series.end(), d);
            _recalcMinMaxMaxMin();
        }

        void removeDataSeries(DataSeries_ptr data_series){
            std::list<SeriesData>::iterator i;
            for(i = m_data_series.begin(); i != m_data_series.end(); i++)
                if(i->data_series == data_series)
                    break;
            m_data_series.erase(i);
            _recalcMinMaxMaxMin();            
        }

        void updateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
            if(std::count_if(m_data_series.begin(), m_data_series.end(), hasInvalidWindow))
                _discardAndUpdateDataWindows(tstart, tend, resolution);
            else
                _incrementalUpdateDataWindows(tstart, tend, resolution);
            
        }

        // Delegated QGraphicsItem Implementation
        QRectF boundingRect() const{
            return m_rect;
        }

        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *option,
                   QWidget *widget){
            const float px_per_unit = option->levelOfDetailFromTransform(painter->worldTransform());
            const float px_width = boundingRect().width() * px_per_unit;
            const float h_resolution_f = std::min(px_width, Graph_Const::Max_Resolution);
            const uint32_t h_resolution = std::max(int(h_resolution_f+0.5), 1);
            const double tend = cauv::nowDouble();
            const double tstart = tend - m_config.time_period;

            //debug() << "graph:paint" << h_resolution << tend-tstart;

            updateDataWindows(tstart, tend, h_resolution);
            
            const QRectF plot_rect = m_axes->contentsRect();

            const float scale_max = m_minimum_maximum > m_data_max?  m_minimum_maximum : m_data_max;
            const float scale_min = m_maximum_minimum < m_data_min?  m_maximum_minimum : m_data_min;
            const float v_units_per_data_unit = plot_rect.height() / (scale_max - scale_min);
            const float v_units_per_second    = double(plot_rect.width()) / (tend - tstart);

            painter->setClipRect(option->exposedRect);

            std::list<SeriesData>::iterator i;
            for(i = m_data_series.begin(); i != m_data_series.end(); i++){
                painter->setBrush(QBrush(QColor(0, 0, 0, 64)));
                painter->setPen(Qt::NoPen);
                painter->drawPolygon(i->data_window->regionAtScale(
                    plot_rect.topLeft(), tstart, scale_max, v_units_per_second, v_units_per_data_unit
                ));
                painter->setPen(QPen(QColor(0, 0, 0, 128)));
                //painter->setBrush(Qt::NoBrush);
                painter->drawPolyline(i->data_window->valuesAtScale(
                    plot_rect.topLeft(), tstart, scale_max, v_units_per_second, v_units_per_data_unit
                ));
            }
            
            painter->setBrush(Qt::NoBrush);
            painter->drawRect(boundingRect().adjusted(1,1,-1,-1));
         }


         // Other drawing-associated Implementation
         void setRect(QRectF const& rect){
            if(rect != m_rect){
                m_rect_changed = true;
                m_rect = rect;
            }
            // axes are always at position zero, so don't need to adjust rect
            m_axes->setRect(rect);
         }
    
    private:
        void _discardAndUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
            std::list<SeriesData>::iterator i;
            for(i = m_data_series.begin(); i != m_data_series.end(); i++)
                i->data_window = i->data_series->m->snapshot(tstart, tend, resolution);
            _updateDataMinMax();
        }

        void _incrementalUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
            std::list<SeriesData>::iterator i;
            for(i = m_data_series.begin(); i != m_data_series.end(); i++)
                i->data_series->m->updateSnapshot(i->data_window, tstart, tend, resolution);
            _updateDataMinMax();            
        }
        
        void _updateDataMinMax(){
            std::list<SeriesData>::iterator i;
            m_data_min = std::numeric_limits<float>::max();
            m_data_max = -std::numeric_limits<float>::max();
            for(i = m_data_series.begin(); i != m_data_series.end(); i++){
                if(i->data_window->min() < m_data_min){
                    if(i->data_window->min() < i->data_series->m->config().minimum_minimum)
                        m_data_min = i->data_series->m->config().minimum_minimum;
                    else
                        m_data_min = i->data_window->min();
                }if(i->data_window->max() > m_data_max && i->data_window->max()){
                    if(i->data_window->max() > i->data_series->m->config().maximum_maximum)
                        m_data_max = i->data_series->m->config().maximum_maximum;
                    else
                        m_data_max = i->data_window->max();
                }
            }
        }


        void _recalcMinMaxMaxMin(){
            std::list<SeriesData>::const_iterator i;
            m_minimum_maximum = -std::numeric_limits<float>::max();
            m_maximum_minimum = std::numeric_limits<float>::max();
            for(i = m_data_series.begin(); i != m_data_series.end(); i++){
                if(i->data_series->m->config().minimum_maximum > m_minimum_maximum)
                    m_minimum_maximum = i->data_series->m->config().minimum_maximum;
                if(i->data_series->m->config().maximum_minimum < m_maximum_minimum)
                    m_maximum_minimum = i->data_series->m->config().maximum_minimum;
            }
        }

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
