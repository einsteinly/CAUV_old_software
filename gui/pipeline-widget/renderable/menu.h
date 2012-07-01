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

