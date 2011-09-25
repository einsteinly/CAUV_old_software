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
            AbstractArcSource *from,
            AbstractArcSink *to);
       
        ArcStyle const& style() const;
        void *source();
        std::set<AbstractArcSink *> sinks();        

        void addTo(AbstractArcSink *to);
        void removeTo(AbstractArcSink *to);

    protected:
        virtual QRectF boundingRect() const;

    protected Q_SLOTS:
        void updateLayout();

    protected:
        ArcStyle const& m_style;
        AbstractArcSource *m_source;
        std::set<AbstractArcSink*> m_sinks;
        QGraphicsPathItem *m_back;
        QGraphicsPathItem *m_front;
};

} // namespace liquid

#endif // __LIQUID_ARC_H__

