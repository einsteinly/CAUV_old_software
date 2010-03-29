#ifndef __BOX_RENDERABLE_H__
#define __BOX_RENDERABLE_H__

#include "draggable.h"


class Box: public Draggable{
    public:
        Box(PipelineWidget& p, double const& w, double const& h)
            : Draggable(p), m_box(0, -h, w, 0){
        }

        virtual void draw(bool){
            if(m_mouseover)
                glColor4f(1.0, 0.0, 0.0, 0.5);
            else
                glColor4f(1.0, 1.0, 1.0, 0.5);
            glBox(m_box);
        }
        
        virtual bool tracksMouse(){
            return true;
        }

        virtual BBox bbox(){
            return m_box;
        }

    private:
        BBox m_box;
};

#endif // ndef __BOX_RENDERABLE_H__

