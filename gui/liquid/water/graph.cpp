/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "graph.h"
#include "internal/graph.h"

namespace w = liquid::water;

w::GraphConfig w::One_Minute = {
    60.0f
};

// - Graph

w::Graph::Graph(GraphConfig const& config, QGraphicsItem* parent)
    : QGraphicsItem(parent),
      m(new w::internal::Graph(config, this)){
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
