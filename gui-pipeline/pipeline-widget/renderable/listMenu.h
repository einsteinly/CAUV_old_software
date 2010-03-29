#ifndef __LIST_MENU_RENDERABLE_H__
#define __LIST_MENU_RENDERABLE_H__

#include <map>
#include <set>
#include <string>
#include <utility>

#include <common/debug.h>

#include "menu.h"
#include "text.h"

template<typename callable>
class ListMenuItem: public Renderable{
    public:
        typedef boost::shared_ptr<callable> callable_ptr;
        ListMenuItem(container_ptr_t c, std::pair<std::string, callable_ptr> const& item)
            : Renderable(c), m_text(boost::make_shared<Text>(c, item.first)),
              m_onclick(item.second), m_bbox(m_text->bbox()),
              m_hovered(false), m_pressed(false){
            // some space around text:
            m_bbox.min.x -= 2.0;
            m_bbox.max.x += 6.0;
            m_bbox.min.y -= 2.0;
            m_bbox.max.y += 4.0;
        }

        virtual void draw(bool picking){
            if(m_pressed)
                glColor(Colour(0.6, 0.9));
            else if(m_hovered)
                glColor(Colour(0.8, 0.8));
            else
                glColor(Colour(0.9, 0.8));
            glBox(m_bbox);
            if(!picking)
                m_text->draw(picking);
        } 
        
        virtual void mouseMoveEvent(MouseEvent const& m){
            if(bbox().contains(m.pos)){
                m_hovered = true;
                if(m.buttons & Qt::LeftButton)
                    m_pressed = true;
            }else{
                m_hovered = false;
                m_pressed = false;
            }
            // TODO: only when necessary
            m_context->postRedraw();
        }

        virtual void mousePressEvent(MouseEvent const& m){
            if(bbox().contains(m.pos)){
                m_hovered = true;
                if(m.buttons & Qt::LeftButton)
                    m_pressed = true;
                // TODO: only when necessary
                m_context->postRedraw();
            }
        }

        virtual void mouseReleaseEvent(MouseEvent const& m){
            if(m_pressed && bbox().contains(m.pos))
                (*m_onclick)();
            m_pressed = false;
            // TODO: only when necessary
            m_context->postRedraw();
        }

        virtual void mouseGoneEvent(){
            m_hovered = false;
            m_pressed = false;
            // TODO: only when necessary
            m_context->postRedraw();
        }

        virtual BBox bbox(){
            return m_bbox;
        }

        void setSpan(double const& xmin, double const& xmax){
            m_bbox.max.x = xmax;
            m_bbox.min.x = xmin;
        }
        void setHeight(double h){
            m_bbox.max.y = m_bbox.min.y + h;
        }

    private:
        boost::shared_ptr<Text> m_text;
        callable_ptr m_onclick;
        BBox m_bbox;

        bool m_hovered;
        bool m_pressed;
};

template<typename callable>
class ListMenu: public Menu{
        typedef ListMenuItem<callable> item_t;
        typedef boost::shared_ptr<item_t> item_ptr;
    public:
        typedef boost::shared_ptr<callable> callable_ptr;
        typedef std::map<std::string,callable_ptr> item_map_t;

        ListMenu(container_ptr_t c, item_map_t const& items)
            : Menu(c), m_items(), m_bbox(){
            typename item_map_t::const_iterator i;
            double y_pos = 0;
            double prev_height = 0;
            for(i = items.begin(); i != items.end(); i++, y_pos -= prev_height){
                item_ptr ip = boost::make_shared<item_t>(c, *i);
                m_items.push_back(ip);                
                ip->m_pos.y = y_pos + ip->bbox().min.y;
                prev_height = ip->bbox().h();
                m_bbox |= ip->bbox() + ip->m_pos;
            }
            typename std::list<item_ptr>::iterator j;
            for(j = m_items.begin(); j != m_items.end(); j++)
                (*j)->setSpan(m_bbox.min.x, m_bbox.max.x);
        }

        virtual void draw(bool picking){
            typename std::list<item_ptr>::const_iterator i;
            /* menus are above: */
            glTranslatef(0, 0, 0.1);
            for(i = m_items.begin(); i != m_items.end(); i++){
                glPushMatrix();
                glTranslatef((*i)->m_pos);
                (*i)->draw(picking);
                glPopMatrix();
            }
        }
        
        virtual void mouseMoveEvent(MouseEvent const& m){
            typename std::list<item_ptr>::iterator i;
            std::set<item_ptr> now_hovered_items;
            for(i = m_items.begin(); i != m_items.end(); i++){
                MouseEvent referred(m, *i);
                if((*i)->bbox().contains(referred.pos)){
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
                if((*i)->bbox().contains(referred.pos)){
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

#endif // ndef __LIST_MENU_RENDERABLE_H__

