/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <list>

#include <boost/shared_ptr.hpp>

#include "util.h"
#include "pwTypes.h"

namespace cauv{
namespace gui{
namespace pw{

class Container{
    protected:
        // protected typedefs
        // NB: THIS MUST REMAIN A LIST: items must be removable without
        // invalidating iterators
        typedef std::list<renderable_ptr_t> renderable_list_t;

    public:
        virtual ~Container(){ }
        
        virtual Point referUp(Point const& p) const = 0;
        virtual void postRedraw(float delay_secs) = 0;
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position,
                              bool pressed=false) = 0;
		virtual void postText(const std::string& text, const std::string& font) = 0;
        virtual void removeMenu(menu_ptr_t) = 0;
        virtual void remove(renderable_ptr_t) = 0;
        virtual renderable_ptr_t pick(Point const& p);
        virtual void refreshLayout(){ };
        
        // draw m_contents
        virtual void draw(drawtype_e::e flags);

        // union of bboxes of m_contents (translated to respective positions),
        // derived types will probably want to override this in order to
        // provide a version that caches the bbox
        virtual BBox bbox();

    protected:
        renderable_list_t m_contents;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __CONTAINER_H__

