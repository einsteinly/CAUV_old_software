#ifndef __MENU_RENDERABLE_H__
#define __MENU_RENDERABLE_H__

#include <map>
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
              m_onclick(item.second), m_bbox(m_text->bbox()){
            // some space around text:
            m_bbox.xmin -= 1.0;
            m_bbox.xmax += 3.0;
            m_bbox.ymin -= 1.0;
            m_bbox.ymax += 2.0;
        }

        virtual void draw(){
            glColor4f(0.9, 0.9, 0.9, 0.8);
            glBegin(GL_QUADS);
            glVertex2f(m_bbox.xmin, m_bbox.ymax);
            glVertex2f(m_bbox.xmin, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymin);
            glVertex2f(m_bbox.xmax, m_bbox.ymax);
            glEnd();
            m_text->draw();
        }

        virtual BBox bbox(){
            return m_bbox;
        }

        void setWidth(double w){
            m_bbox.xmax = m_bbox.xmin + w;
        }
        void setHeight(double h){
            m_bbox.ymax = m_bbox.ymin + h;
        }

    private:
        boost::shared_ptr<Text> m_text;
        callable_ptr m_onclick;
        BBox m_bbox;
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
            double max_width = 0;
            double y_pos = 0;
            double previous_height = 0;
            for(i = items.begin(); i != items.end(); i++){
                item_ptr ip = boost::make_shared<item_t>(boost::ref(p), *i);
                m_items.push_back(ip);
                ip->m_pos_y = y_pos + ip->bbox().ymin;
                y_pos -= previous_height;
                previous_height = ip->bbox().ymax - ip->bbox().ymin;
                double width = 0.0;
                if((width = ip->bbox().xmax - ip->bbox().xmin) > max_width)
                    max_width = width;
            }
            typename std::list<item_ptr>::iterator j;
            for(j = m_items.begin(); j != m_items.end(); j++)
                (*j)->setWidth(max_width);
            
        }

        virtual void draw(){
            typename std::list<item_ptr>::const_iterator i;
            for(i = m_items.begin(); i != m_items.end(); i++){
                glPushMatrix();
                glTranslatef((*i)->m_pos_x, (*i)->m_pos_y, 0);
                (*i)->draw();
                glPopMatrix();
            }
        }

        virtual bool tracksMouse(){ return true; }

    private:
        std::list<item_ptr> m_items;
};

#endif // ndef __MENU_RENDERABLE_H__

