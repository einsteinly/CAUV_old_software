/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include <boost/enable_shared_from_this.hpp>

#include "../renderable.h"

namespace cauv{
namespace gui{
namespace pw{

class Menu: public Renderable, public boost::enable_shared_from_this<Menu>{
    public:
        Menu(container_ptr_t c) : Renderable(c){ }
        virtual ~Menu(){ }

        virtual Point topLevelPos() const{ return m_pos; }
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __MENU_RENDERABLE_H__

