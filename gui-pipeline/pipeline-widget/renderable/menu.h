#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include "../renderable.h"

class Menu: public Renderable, boost::enable_shared_from_this<Menu>{
    public:
        Menu(PipelineWidget& p) : Renderable(p){ }
        virtual bool keyPressEvent(QKeyEvent*){ return false; }
        virtual bool keyReleaseEvent(QKeyEvent*){ return false; }
};


#endif // ndef __MENU_RENDERABLE_H__

