/* Copyright 2011 Cambridge Hydronautics Ltd.
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

