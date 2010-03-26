#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include "../renderable.h"


class Box: public Renderable{
    public:
        Box(PipelineWidget& p, double w, double h)
            : Renderable(p), m_width(w), m_height(h), m_click_pos_x(0),
              m_click_pos_y(0), m_mouseover(false){
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

        virtual void mouseMoveEvent(MouseEvent const& event){
            if(event.buttons & Qt::LeftButton){
                m_pos_x += event.x - m_click_pos_x;
                m_pos_y += event.y - m_click_pos_y;
                // need a re-draw
                m_parent.updateGL();
            }else if(!m_mouseover){
                m_mouseover = true;
                m_parent.updateGL();
            }
        }

        virtual void mousePressEvent(MouseEvent const& event){
            m_click_pos_x = event.x;
            m_click_pos_y = event.y;
        }

        virtual void mouseGoneEvent(){
            m_mouseover = false;
            m_parent.updateGL();
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

        double m_click_pos_x;
        double m_click_pos_y;

        bool m_mouseover;
};

#endif // ndef __MENU_RENDERABLE_H__

