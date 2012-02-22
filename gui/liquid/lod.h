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
#ifndef __LIQUID_LEVEL_OF_DETAIL_H__
#define __LIQUID_LEVEL_OF_DETAIL_H__

namespace liquid{

//... still thinking about this interface

template<typename T>
class LODItem;

/*
namespace LevelOfDetail{
enum e{
    Full, Greek, Shape, Rect
};
} // namespace LevelOfDetail

class LODCapable{
    protected:
        typedef int must_inherit_LODCapable;
    
        virtual void setPainterFor(LevelOfDetail::e lod, QPainter* painter) const = 0;
};
*/

} // namespace liquid

#endif // ndef __LIQUID_LEVEL_OF_DETAIL_H__
