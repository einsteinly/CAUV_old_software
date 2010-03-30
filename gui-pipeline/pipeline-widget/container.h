#ifndef __CONTAINER_H__
#define __CONTAINER_H__

#include <list>

#include <boost/shared_ptr.hpp>

#include "util.h"

class Renderable;
class Menu;

class Container{
    public:
        // public typedefs
        typedef boost::shared_ptr<Renderable> renderable_ptr_t;
        typedef boost::shared_ptr<Menu> menu_ptr_t;

    protected:
        // protected typedefs
        // NB: THIS MUST REMAIN A LIST: items must be removable without
        // invalidating iterstors
        typedef std::list<renderable_ptr_t> renderable_list_t;

    public:
        virtual Point referUp(Point const& p) const = 0;
        virtual void postRedraw() = 0;
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position) = 0;
        virtual void removeMenu(menu_ptr_t) = 0;
        
        // draw m_contents
        virtual void draw(bool picking);

        // union of bboxes of m_contents (translated to respective positions),
        // derived types will probably want to override this in order to
        // provide a version that caches the bbox
        virtual BBox bbox();

    protected:
        renderable_list_t m_contents;
};


#endif // ndef __CONTAINER_H__

