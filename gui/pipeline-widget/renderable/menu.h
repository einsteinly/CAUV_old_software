#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include <boost/enable_shared_from_this.hpp>

#include "../renderable.h"

namespace pw{

class Menu: public Renderable, public boost::enable_shared_from_this<Menu>{
    public:
        Menu(container_ptr_t c) : Renderable(c){ }
        virtual ~Menu(){ }

        virtual bool keyPressEvent(QKeyEvent*){ return false; }
        virtual bool keyReleaseEvent(QKeyEvent*){ return false; }
        virtual Point topLevelPos() const{ return m_pos; }
};

} // namespace pw

#endif // ndef __MENU_RENDERABLE_H__
