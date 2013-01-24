/* Copyright 2012-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_WATER_INTERNAL_GRAPH_AXES_H__
#define __LIQUID_WATER_INTERNAL_GRAPH_AXES_H__

#include <QGraphicsItem>
#include <QRectF>
#include <QString>

namespace liquid{
namespace water{
namespace internal{

class GraphLegend: public QGraphicsItem{
};

class GraphAxes: public QGraphicsItem{
    public:
        GraphAxes(QRectF const& rect, QGraphicsItem* parent);

        QRectF contentsRectAtScale(float px_per_unit) const;

        void setRect(QRectF const& rect);

        void setScales(float const& xmin, float const& xmax,
                       float const& ymin, float const& ymax);

        // QGraphicsItem Implementation:
        QRectF boundingRect() const;

        void paint(QPainter *painter,
                   const QStyleOptionGraphicsItem *option,
                   QWidget *widget);
    
    private:
        enum{
            Bare_Width = 90,
            Bare_Height = 60,
            Spartan_Width = 270,
            Spartan_Height = 180
        };
        QRectF m_rect;

        float m_xmin;
        float m_xmax;
        float m_ymin;
        float m_ymax;
};

} // namespace internal
} // namespace water
} // namespace liquid

#endif // ndef __LIQUID_WATER_INTERNAL_GRAPH_AXES_H__
