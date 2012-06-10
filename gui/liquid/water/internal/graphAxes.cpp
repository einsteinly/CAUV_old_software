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

#include "graphAxes.h"

#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QPen>
#include <QBrush>

namespace w = liquid::water;
namespace wi = liquid::water::internal;

wi::GraphAxes::GraphAxes(QRectF const& rect, QGraphicsItem* parent)
    : QGraphicsItem(parent),
      m_rect(rect),
      m_xmin(0),
      m_xmax(0),
      m_ymin(0),
      m_ymax(0){
}

QRectF wi::GraphAxes::contentsRectAtScale(float px_per_unit) const{
    const int px_width = px_per_unit * m_rect.width();
    const int px_height = px_per_unit * m_rect.height();
    if(px_width < Bare_Width || px_height < Bare_Height){
        return m_rect;
    }else if(px_width < Spartan_Width || px_height < Spartan_Height){
        return m_rect.adjusted(36, 0, 0, 0);
    }else{
        return m_rect.adjusted(48, 0, 0, -16);
    }
    return m_rect;
}

void wi::GraphAxes::setRect(QRectF const& rect){
    prepareGeometryChange();
    m_rect = rect;
}

void wi::GraphAxes::setScales(float const& xmin, float const& xmax,
                              float const& ymin, float const& ymax){
    m_xmin = xmin;
    m_xmax = xmax;
    m_ymin = ymin;
    m_ymax = ymax;
}

// QGraphicsItem Implementation:
QRectF wi::GraphAxes::boundingRect() const{
    return m_rect;
}

void wi::GraphAxes::paint(QPainter *painter,
                          const QStyleOptionGraphicsItem *option,
                          QWidget *widget){
    Q_UNUSED(widget);

    painter->setClipRect(m_rect);
    
    painter->setPen(QPen(QColor(0, 0, 0, 128)));
    painter->setBrush(Qt::NoBrush);
    QFont smallfont;
    smallfont.setPointSize(10);
    painter->setFont(smallfont);

    QFontMetrics font_metrics(painter->font());

    const float px_per_unit = option->levelOfDetailFromTransform(painter->worldTransform());
    const QRectF cr = contentsRectAtScale(px_per_unit);
    painter->drawRect(cr);
    const float y_labels_space = cr.left() - m_rect.left();
    const float x_labels_space = m_rect.bottom() - cr.bottom();
    //const float title_space = cr.top() - m_rect.top();
    
    // !!! TODO change df for x and y separately based on the width and
    // height of the graph. Or, even better, the axes labels should be
    // pinned to 'interesting' values. Maybe allow a function to be
    // supplied in the series config to pick interesting values.
    const float df = 0.5;
    float pos_adjust = 0.0;
    if(y_labels_space >= 48){
        for(float frac = 0; frac <= 1.0001; frac += df){
            QString label = QString("%1").arg(m_ymin + frac * (m_ymax-m_ymin), 7, 'f', 3);
            if(frac == 0)
                pos_adjust = -font_metrics.ascent()/2;
            else if(frac > 0.999)
                pos_adjust = font_metrics.ascent();
            else
                pos_adjust = font_metrics.ascent()/2;
            painter->drawText(
                cr.bottomLeft() - QPointF(y_labels_space, cr.height() * frac - pos_adjust), label
            );
        }
    }else if(y_labels_space >= 36){
       for(float frac = 0; frac <= 1.0001; frac += df){
            QString label = QString("%1").arg(m_ymin + frac * (m_ymax-m_ymin), 7, 'f', 1);
            if(frac == 0)
                pos_adjust = -font_metrics.ascent()/2;
            else if(frac > 0.999)
                pos_adjust = font_metrics.ascent();
            else
                pos_adjust = font_metrics.ascent()/2;
            painter->drawText(
                cr.bottomLeft() - QPointF(y_labels_space, cr.height() * frac - pos_adjust), label
            );
        }
    }

    if(x_labels_space >= 16){
        for(float frac = 0; frac <= 1.0001; frac += df){
            QString label = QString("%1").arg(m_xmin + frac * (m_xmax-m_xmin), 7, 'f', 1);
            if(frac == 0)
                pos_adjust = 0;
            else if(frac > 0.999)
                pos_adjust = font_metrics.width(label);
            else
                pos_adjust = font_metrics.width(label)/2;
            painter->drawText(
                cr.bottomLeft() + QPointF(cr.width()*frac - pos_adjust, x_labels_space-1), label
            );
        }
    }

    /*if(title_space >= 20){
        pos_adjust = font_metrics.width(m_title)/2;
        painter->drawText(
            cr.topLeft() + QPointF(cr.width()/2-pos_adjust, -8), m_title
        );
    }*/

}
       
