/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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
    
        virtual void setPainterFor(LevelOfDetail lod, QPainter* painter) const = 0;
};
*/

} // namespace liquid

#endif // ndef __LIQUID_LEVEL_OF_DETAIL_H__
