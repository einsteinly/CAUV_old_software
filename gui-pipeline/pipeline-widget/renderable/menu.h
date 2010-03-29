#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include "../renderable.h"

class Menu: public Renderable{
    public:
        Menu(container_ptr_t c) : Renderable(c){ }
        virtual bool keyPressEvent(QKeyEvent*){ return false; }
        virtual bool keyReleaseEvent(QKeyEvent*){ return false; }
};


#endif // ndef __MENU_RENDERABLE_H__

