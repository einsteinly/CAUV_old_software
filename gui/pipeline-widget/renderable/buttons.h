/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __BUTTONS_RENDERABLES_H__
#define __BUTTONS_RENDERABLES_H__

#include "node.h"

namespace cauv{
namespace gui{
namespace pw{

class Button: public Renderable{
    public:
        Button(container_ptr_t c, float size=12.0f)
            : Renderable(c), m_size(size), m_mouseover(false),
              m_pressed(false){
        }
        virtual ~Button(){ }
        
        virtual void draw(drawtype_e::e){
            if(m_pressed)
                glColor(Colour(1, 1));
            else if(m_mouseover)
                glColor(Colour(1, 0.5));
            else
                glColor(Colour(1, 0.2));

            glBox(bbox(), 3);
        }

        virtual void onClick() = 0;

        virtual void mouseMoveEvent(MouseEvent const& event){
            if(bbox().contains(event.pos) && !m_mouseover){
                m_mouseover = true;
                m_context->postRedraw(0);
            }
        }

        virtual bool mousePressEvent(MouseEvent const& event){
            if(bbox().contains(event.pos)){
                m_pressed = true;
                m_context->postRedraw(0);
                return true;
            }
            return false;
        }

        virtual void mouseReleaseEvent(MouseEvent const& event){
            if(m_pressed && bbox().contains(event.pos)){
                this->onClick();
            }
            m_pressed = false;
            m_context->postRedraw(0);
        }

        virtual void mouseGoneEvent(){
            if(m_pressed || m_mouseover){
                m_pressed = false;
                m_mouseover = false;
                m_context->postRedraw(0);
            }
        }

        virtual bool tracksMouse(){
            return true;
        }

        virtual BBox bbox(){
            return BBox(-m_size/2, -m_size/2, m_size/2, m_size/2);
        }
    protected:
        double m_size;
        bool m_mouseover;
        bool m_pressed;
};

template<typename container_T>
class CloseButton: public Button{
    public:
        CloseButton(container_T* c)
            : Button(c), m_container(c){
        }
        virtual ~CloseButton(){ }

        virtual void draw(drawtype_e::e flags){
            Button::draw(flags);

            if(m_pressed)
                glColor(Colour(0, 1));
            else if(m_mouseover)
                glColor(Colour(0.8, 0, 0, 0.8));
            else
                glColor(Colour(0, 0.5));
            
            // TODO: don't draw this with lines - it looks ugly...
            glLineWidth(roundA(m_size/8));
            glBegin(GL_LINES);
            glVertex2f(2 - m_size/2, m_size/2 - 2);
            glVertex2f(m_size/2 - 2, 2 - m_size/2);
            glVertex2f(2 - m_size/2, 2 - m_size/2);
            glVertex2f(m_size/2 - 2, m_size/2 - 2);
            glEnd();
        }

        virtual void onClick(){
            m_container->close();
        }

     private:
        container_T* m_container;
};

class ExecButton: public Button{
    public:
        ExecButton(Node* c)
            : Button(c), m_node(c){
        }
        virtual ~ExecButton(){ }

        virtual void draw(drawtype_e::e flags){
            Button::draw(flags);

            if(m_pressed)
                glColor(Colour(0, 1));
            else if(m_mouseover)
                glColor(Colour(0, 0.6, 0, 0.8));
            else
                glColor(Colour(0, 0.5));
            
            // TODO: don't draw this with lines - it looks ugly...
            glLineWidth(roundA(m_size/8));
            const float radius = m_size/2 - 3;
            glArc(radius, -45, 180, 16);
            glBegin(GL_TRIANGLES);
            glVertex2f(0, -2.5-radius);
            glVertex2f(0, 2.5-radius);
            glVertex2f(-4, -radius);
            glEnd();
        }

        virtual void onClick(){
            m_node->exec();
        }
    
     private:
        Node* m_node;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __BUTTONS_RENDERABLES_H__
