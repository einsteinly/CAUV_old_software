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

#include "graph.h"

#include <boost/make_shared.hpp>

#include <QStyleOptionGraphicsItem>
#include <QPainter>
#include <QPolygonF>
#include <QPen>
#include <QBrush>
#include <QGraphicsScene>

#include <debug/cauv_debug.h>

#include <utility/time.h>
#include <utility/math.h>

#include "../graph.h"
#include "dataSeries.h"
#include "dataWindow.h"
#include "graphAxes.h"


namespace w = liquid::water;
namespace wi = liquid::water::internal;

wi::Graph::Graph(GraphConfig const& config, QGraphicsItem* owner)
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

void wi::Graph::addDataSeries(DataSeries_ptr data_series){
    SeriesData d = {data_series, DataWindow_ptr()};
    m_data_series.insert(m_data_series.end(), d);
    data_series->m->addGraph(this);
    _recalcMinMaxMaxMin();
}

void wi::Graph::removeDataSeries(DataSeries_ptr data_series){
    std::list<SeriesData>::iterator i;
    for(i = m_data_series.begin(); i != m_data_series.end(); i++)
        if(i->data_series == data_series)
            break;
    m_data_series.erase(i);
    _recalcMinMaxMaxMin();            
}

void wi::Graph::updateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
    if(std::count_if(m_data_series.begin(), m_data_series.end(), hasInvalidWindow))
        _discardAndUpdateDataWindows(tstart, tend, resolution);
    else
        _incrementalUpdateDataWindows(tstart, tend, resolution);
    
}

// Delegated QGraphicsItem Implementation
QRectF wi::Graph::boundingRect() const{
    return m_rect;
}

void wi::Graph::requiresUpdate() const{
    QGraphicsScene* s = m_owner->scene();
    if(s)
        s->update((m_owner->mapToScene(boundingRect())).boundingRect());
}

void wi::Graph::paint(QPainter *painter,
           const QStyleOptionGraphicsItem *option,
           QWidget *widget){
    Q_UNUSED(widget);

    const float px_per_unit = option->levelOfDetailFromTransform(painter->worldTransform());
    const float px_width = boundingRect().width() * px_per_unit;
    const float h_resolution_f = std::min(px_width, Graph_Const::Max_Resolution);
    const uint32_t h_resolution = std::max(int(h_resolution_f+0.5), 1);
    const double tend = cauv::nowDouble();
    const double tstart = tend - m_config.time_period;

    //debug() << "graph:paint" << h_resolution << tend-tstart;

    updateDataWindows(tstart, tend, h_resolution);
    
    const QRectF plot_rect = m_axes->contentsRectAtScale(px_per_unit);

    const float scale_max = m_minimum_maximum > m_data_max?  m_minimum_maximum : m_data_max;
    const float scale_min = m_maximum_minimum < m_data_min?  m_maximum_minimum : m_data_min;
    const float v_units_per_data_unit = plot_rect.height() / (scale_max - scale_min);
    const float v_units_per_second    = double(plot_rect.width()) / (tend - tstart);
    
    m_axes->setScales(-m_config.time_period, 0, scale_min, scale_max);

    painter->setClipRect(plot_rect);

    std::list<SeriesData>::iterator i;
    for(i = m_data_series.begin(); i != m_data_series.end(); i++){
        const QPolygonF poly = i->data_window->regionAtScale(
            plot_rect.topLeft(), tstart, scale_max, v_units_per_second, v_units_per_data_unit
        );
        if(poly.size()){
            painter->setBrush(QBrush(QColor(0, 0, 0, 64)));
            painter->setPen(Qt::NoPen);
            painter->drawPolygon(poly);
        }
        painter->setPen(QPen(QColor(0, 0, 0, 128)));
        painter->setBrush(Qt::NoBrush);
        painter->drawPolyline(i->data_window->valuesAtScale(
            plot_rect.topLeft(), tstart, scale_max, v_units_per_second, v_units_per_data_unit
        ));
    }
    
    painter->setBrush(Qt::NoBrush);
 }


 // Other drawing-associated Implementation
 void wi::Graph::setRect(QRectF const& rect){
    if(rect != m_rect){
        m_rect_changed = true;
        m_rect = rect;
    }
    // axes are always at position zero, so don't need to adjust rect
    m_axes->setRect(rect);
 }

void wi::Graph::_discardAndUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
    std::list<SeriesData>::iterator i;
    for(i = m_data_series.begin(); i != m_data_series.end(); i++)
        i->data_window = i->data_series->m->snapshot(tstart, tend, resolution, this);
    _updateDataMinMax();
}

void wi::Graph::_incrementalUpdateDataWindows(double const& tstart, double const& tend, uint32_t resolution){
    std::list<SeriesData>::iterator i;
    for(i = m_data_series.begin(); i != m_data_series.end(); i++)
        i->data_series->m->updateSnapshot(i->data_window, tstart, tend, resolution, this);
    _updateDataMinMax();            
}

void wi::Graph::_updateDataMinMax(){
    std::list<SeriesData>::iterator i;
    m_data_min = std::numeric_limits<float>::max();
    m_data_max = -std::numeric_limits<float>::max();
    for(i = m_data_series.begin(); i != m_data_series.end(); i++){
        if(!i->data_window->size())
            continue;
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
    //debug() << __func__ << "min =" << m_data_min << "max = " << m_data_max;            
}


void wi::Graph::_recalcMinMaxMaxMin(){
    std::list<SeriesData>::const_iterator i;
    m_minimum_maximum = -std::numeric_limits<float>::max();
    m_maximum_minimum = std::numeric_limits<float>::max();
    for(i = m_data_series.begin(); i != m_data_series.end(); i++){
        if(i->data_series->m->config().minimum_maximum > m_minimum_maximum)
            m_minimum_maximum = i->data_series->m->config().minimum_maximum;
        if(i->data_series->m->config().maximum_minimum < m_maximum_minimum)
            m_maximum_minimum = i->data_series->m->config().maximum_minimum;
    }
    //debug() << __func__ << "min max =" << m_minimum_maximum << "max min = " << m_maximum_minimum;
}


bool wi::Graph::hasInvalidWindow(SeriesData const& d){
    return !d.data_window || !d.data_window->size();
}


