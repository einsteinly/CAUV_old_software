#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include <map>
#include <set>
#include <string>
#include <utility>

#include <common/debug.h>

#include "../renderable.h"
#include "text.h"

template<typename callable>
class MenuItem: public Renderable{
    public:
        typedef boost::shared_ptr<callable> callable_ptr;
        MenuItem(PipelineWidget& p, std::pair<std::string, callable_ptr> const& item)
            : Renderable(p), m_text(boost::make_shared<Text>(boost::ref(p), item.first)),
              m_onclick(item.second), m_bbox(m_text->bbox()),
              m_hovered(false), m_pressed(false){
            // some space around text:
            m_bbox.xmin -= 2.0;
            m_bbox.xmax += 6.0;
            m_bbox.ymin -= 2.0;
            m_bbox.ymax += 4.0;
        }

        virtual void draw(bool picking){
            if(m_pressed)
                glColor4f(0.6, 0.6, 0.6, 0.9);
            else if(m_hovered)
                glColor4f(0.8, 0.8, 0.8, 0.8);
            else
                glColor4f(0.9, 0.9, 0.9, 0.9);
            glBegin(GL_QUADS);
            glVertex2f(m_bbox.xmin, m_bbox.ymax);
            glVertex2f(m_bbox.xmin, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymax);
            glEnd();
            if(!picking)
                m_text->draw(picking);
        } 
        
        virtual void mouseMoveEvent(MouseEvent const& m){
            if(bbox().contains(m.x, m.y)){
                m_hovered = true;
                if(m.buttons & Qt::LeftButton)
                    m_pressed = true;
            }else{
                m_hovered = false;
                m_pressed = false;
            }
            // TODO: only when necessary
            m_parent.updateGL();
        }

        virtual void mousePressEvent(MouseEvent const& m){
            if(bbox().contains(m.x, m.y)){
                m_hovered = true;
                if(m.buttons & Qt::LeftButton)
                    m_pressed = true;
                // TODO: only when necessary
                m_parent.updateGL();
            }
        }

        virtual void mouseReleaseEvent(MouseEvent const& m){
            if(m_pressed && bbox().contains(m.x, m.y))
                (*m_onclick)();
            m_pressed = false;
            // TODO: only when necessary
            m_parent.updateGL();
        }

        virtual void mouseGoneEvent(){
            m_hovered = false;
            m_pressed = false;
            // TODO: only when necessary
            m_parent.updateGL();
        }

        virtual BBox bbox(){
            return m_bbox;
        }

        void setSpan(double const& xmin, double const& xmax){
            m_bbox.xmax = xmax;
            m_bbox.xmin = xmin;
        }
        void setHeight(double h){
            m_bbox.ymax = m_bbox.ymin + h;
        }

    private:
        boost::shared_ptr<Text> m_text;
        callable_ptr m_onclick;
        BBox m_bbox;

        bool m_hovered;
        bool m_pressed;
};

template<typename callable>
class Menu: public Renderable{
        typedef MenuItem<callable> item_t;
        typedef boost::shared_ptr<item_t> item_ptr;
    public:
        typedef boost::shared_ptr<callable> callable_ptr;
        typedef std::map<std::string,callable_ptr> item_map_t;

        Menu(PipelineWidget& p, item_map_t const& items)
            : Renderable(p), m_items(){
            typename item_map_t::const_iterator i;
            double max_x = 0;
            double min_x = 0;
            double y_pos = 0;
            double previous_height = 0;
            for(i = items.begin(); i != items.end(); i++){
                item_ptr ip = boost::make_shared<item_t>(boost::ref(p), *i);
                m_items.push_back(ip);
                ip->m_pos_y = y_pos + ip->bbox().ymin;
                y_pos -= previous_height;
                previous_height = ip->bbox().ymax - ip->bbox().ymin;
                if(ip->bbox().xmax > max_x)
                    max_x = ip->bbox().xmax;
                if(ip->bbox().xmin < min_x)
                    min_x = ip->bbox().xmin;
            }
            typename std::list<item_ptr>::iterator j;
            for(j = m_items.begin(); j != m_items.end(); j++)
                (*j)->setSpan(min_x, max_x);
            m_bbox.xmin = min_x;
            m_bbox.xmax = max_x;
            if(m_items.size()){
                m_bbox.ymin = m_items.back()->bbox().ymin + y_pos;
                m_bbox.ymax = m_items.front()->bbox().ymax; 
            }
        }

        virtual void draw(bool picking){
            typename std::list<item_ptr>::const_iterator i;
            for(i = m_items.begin(); i != m_items.end(); i++){
                glPushMatrix();
                glTranslatef((*i)->m_pos_x, (*i)->m_pos_y, 0);
                (*i)->draw(picking);
                glPopMatrix();
            }
        }
        
        virtual void mouseMoveEvent(MouseEvent const& m){
            typename std::list<item_ptr>::iterator i;
            std::set<item_ptr> now_hovered_items;
            for(i = m_items.begin(); i != m_items.end(); i++){
                MouseEvent referred(m, *i);
                if((*i)->bbox().contains(referred.x, referred.y)){
                    (*i)->mouseMoveEvent(referred);
                    now_hovered_items.insert(*i);
                }
            }
            typename std::set<item_ptr>::iterator j;            
            for(j = m_hovered_items.begin(); j != m_hovered_items.end(); j++)
                if(!now_hovered_items.count(*j))
                    (*j)->mouseGoneEvent();
            m_hovered_items = now_hovered_items;
        }

        virtual void mousePressEvent(MouseEvent const& m){
            typename std::list<item_ptr>::iterator i;
            for(i = m_items.begin(); i != m_items.end(); i++){
                MouseEvent referred(m, *i);
                if((*i)->bbox().contains(referred.x, referred.y)){
                    (*i)->mousePressEvent(referred);
                    m_pressed_items.insert(*i);
                }
            }
        }

        virtual void mouseReleaseEvent(MouseEvent const& m){
            typename std::set<item_ptr>::iterator i;
            for(i = m_pressed_items.begin(); i != m_pressed_items.end(); i++)
                (*i)->mouseReleaseEvent(MouseEvent(m, *i));
            m_pressed_items.clear();
        }

        virtual void mouseGoneEvent(){
            typename std::set<item_ptr>::iterator i;
            for(i = m_hovered_items.begin(); i != m_hovered_items.end(); i++)
                (*i)->mouseGoneEvent();
            m_hovered_items.clear();
        }

        virtual bool tracksMouse(){ return true; }

        BBox bbox(){
            return m_bbox;
        }

    private:
        std::list<item_ptr> m_items;
        BBox m_bbox;

        std::set<item_ptr> m_hovered_items;
        std::set<item_ptr> m_pressed_items;
};

#endif // ndef __MENU_RENDERABLE_H__

