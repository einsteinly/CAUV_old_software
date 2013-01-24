/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __LIQUID_REQUIRES_CUTOUT_H__
#define __LIQUID_REQUIRES_CUTOUT_H__

#include "style.h"

#include <QGraphicsItem>

namespace liquid{

class RequiresCutout{
    public:
        virtual ~RequiresCutout(){ }
        virtual QList<CutoutStyle> cutoutGeometry() const = 0;
        virtual QGraphicsItem const* asQGI() const{
            return dynamic_cast<QGraphicsItem const*>(this);
        }
};

} // namespace liquid

#endif // ndef __LIQUID_REQUIRES_CUTOUT_H__

