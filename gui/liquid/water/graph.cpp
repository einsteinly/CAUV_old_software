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
#include "internal/graph.h"

namespace w = liquid::water;

// - DataSeries

w::DataSeries::DataSeries(SeriesConfig const& config, QString series_name)
    : m(new internal::DataSeries(config, series_name)){
}

void w::DataSeries::postData(double const& value, double const& time){
    m->postData(value, time);
}


// - Graph

w::Graph::Graph(GraphConfig const& config, QString name, QGraphicsItem* parent)
    : QGraphicsItem(parent),
      m(new w::internal::Graph(config, name, this)){
}

void w::Graph::addDataSeries(DataSeries_ptr data_series){
    m->addDataSeries(data_series);
}

void w::Graph::setRect(QRectF const& r){
    prepareGeometryChange();
    m->setRect(r);
}

// - Graph::QGraphicsItem implementation
QRectF w::Graph::boundingRect() const{
    return m->boundingRect();
}


void w::Graph::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget){
    m->paint(painter, option, widget);
}
