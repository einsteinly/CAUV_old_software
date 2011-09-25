#ifndef __LIQUID_REQUIRES_CUTOUT_H__
#define __LIQUID_REQUIRES_CUTOUT_H__

#include "style.h"

namespace liquid{

class RequiresCutout{
    public:
        virtual QList<CutoutStyle> cutoutGeometry() const = 0;
        virtual QGraphicsItem const* asQGI() const{
            return dynamic_cast<QGraphicsItem const*>(this);
        }
};

} // namespace liquid

#endif // ndef __LIQUID_REQUIRES_CUTOUT_H__

