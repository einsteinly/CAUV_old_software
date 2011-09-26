#ifndef __LIQUID_ARC_H__
#define __LIQUID_ARC_H__

#include <set>

#include "arcSource.h"

namespace liquid {

struct ArcStyle;
class AbstractArcSink;

class Arc: public AbstractArcSource{
        Q_OBJECT;
    public:
        Arc(ArcStyle const& of_style,
            AbstractArcSource *from=NULL,
            AbstractArcSink *to=NULL);
       
        ArcStyle const& style() const;
        void *source();
        std::set<AbstractArcSink *> sinks();        
        
        void setFrom(AbstractArcSource *from);
        void addTo(AbstractArcSink *to);

    protected:
        virtual QRectF boundingRect() const;
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget = 0);

    protected Q_SLOTS:
        void updateLayout();
        void removeTo(AbstractArcSink *to);

    protected:
        ArcStyle const& m_style;
        AbstractArcSource *m_source;
        std::set<AbstractArcSink*> m_sinks;
        QGraphicsPathItem *m_back;
        QGraphicsPathItem *m_front;
};

} // namespace liquid

#endif // __LIQUID_ARC_H__

