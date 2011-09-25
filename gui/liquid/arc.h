#ifndef __LIQUID_ARC_H__
#define __LIQUID_ARC_H__

#include <set>

#include <QGraphicsArcItem>

#include "arcSource.h"

namespace liquid {

struct ArcStyle;
struct ArcSink;

class Arc: public AbstractArcSource{
        Q_OBJECT;
    public:
        Arc(ArcStyle const& of_style,
            AbstractArcSource *from,
            ArcSink *to);
       
        ArcStyle const& style() const;
        void *source();
        std::set<ArcSink *> sinks();        

        void addTo(ArcSink *to);
        void removeTo(ArcSink *to);

    protected:
        virtual QRectF boundingRect() const;

    protected Q_SLOTS:
        void updateLayout();

    protected:
        ArcStyle const& m_style;
        AbstractArcSource *m_source;
        std::set<ArcSink*> m_sinks;
        QGraphicsPathItem *m_front;
        QGraphicsPathItem *m_back;
};

} // namespace liquid

#endif // __LIQUID_ARC_H__

