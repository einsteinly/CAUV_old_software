#ifndef __BOX_RENDERABLE_H__
#define __BOX_RENDERABLE_H__

#include "draggable.h"


class Box: public Draggable{
    public:
        Box(PipelineWidget& p, double w, double h)
            : Draggable(p), m_width(w), m_height(h){
        }

        virtual void draw(bool){
            if(m_mouseover)
                glColor4f(1.0, 0.0, 0.0, 0.5);
            else
                glColor4f(1.0, 1.0, 1.0, 0.5);
            glBegin(GL_QUADS);
            glVertex2f(0, 0);
            glVertex2f(0, -m_height);
            glVertex2f(m_width, -m_height);
            glVertex2f(m_width, 0);
            glEnd();
        }
        
        virtual bool tracksMouse(){
            return true;
        }

        virtual BBox bbox(){
            BBox r = {0, -m_height, m_width, 0};
            return r;
        }

    private:
        double m_width;
        double m_height;
};

#endif // ndef __BOX_RENDERABLE_H__

