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
