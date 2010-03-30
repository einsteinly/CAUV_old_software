#ifndef __ARC_RENDERABLE_H__
#define __ARC_RENDERABLE_H__

#include "../renderable.h"

class Node;

// Arcs draw their stuff in top level coordinates, so they can't be used in
// containers
class Arc: public Renderable{
    public:
        typedef boost::shared_ptr<Renderable> renderable_ptr_t;

        Arc(container_ptr_t c, renderable_ptr_t src, renderable_ptr_t dst)
            : Renderable(c), m_src(src), m_dst(dst){
        }

        virtual void draw(bool picking){
            if(picking) return;
            
            // TODO: what level should arcs be drawn at?
            glTranslatef(0, 0, 0.05);

            glColor(Colour(1.0, 0.8));
            glLineWidth(1);
            glBegin(GL_LINES);
            glVertex(m_src->topLevelPos());
            glVertex(m_dst->topLevelPos());
            glEnd();

            glTranslatef(m_src->topLevelPos());
            glCircle(3.0);
            glCircleOutline(3.0);

            glTranslatef(m_dst->topLevelPos() - m_src->topLevelPos());
            glCircle(3.0);
            glCircleOutline(3.0);
        }
        
        virtual bool acceptsMouseEvents(){
            return false;
        }

        renderable_ptr_t m_src;
        renderable_ptr_t m_dst;
};


#endif // ndef __ARC_RENDERABLE_H__

