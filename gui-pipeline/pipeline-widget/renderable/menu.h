#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include "../renderable.h"

#include <map>

template<typename callable>
class Menu: public Renderable{
    public:
        Menu(PipelineWidget& p, std::map<std::string, callable> const& items)
            : Renderable(p), m_items(items){
        }

        virtual void draw(){
        }

        virtual bool tracksMouse(){ return true; }

    private:
        std::map<std::string, callable> m_items;
};

#endif // ndef __MENU_RENDERABLE_H__

