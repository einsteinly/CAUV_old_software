#ifndef __LIQUID_ARC_H__
#define __LIQUID_ARC_H__

#include "arcStyle.h"
#include "arcSource.h"

namespace liquid {

struct ArcStyle;

class Arc : public ArcSource {
    public:
        Arc(ArcStyle const& of_style);
       
        void *source();
        std::list<ArcSink *> sinks();
};

} // namespace liquid

#endif // __LIQUID_ARC_H__

