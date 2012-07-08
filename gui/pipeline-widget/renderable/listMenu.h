/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 * 
 * See license.txt for details.
 * 
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#ifndef __LIST_MENU_RENDERABLE_H__
#define __LIST_MENU_RENDERABLE_H__

#include <map>
#include <set>
#include <string>
#include <utility>
#include <algorithm>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <debug/cauv_debug.h>

#include "menu.h"
#include "text.h"

namespace cauv{
namespace gui{
namespace pw{

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

        virtual ~ListMenuItem(){ }

        virtual void draw(drawtype_e::e flags){
            if(m_pressed)
                glColor(Colour(0.50, 0.8));
            else if(m_hovered)
                glColor(Colour(0.75, 0.8));
            else
                glColor(Colour(1.00, 0.5));
            glBox(m_bbox);
            if(!(flags & drawtype_e::picking))
                m_text->draw(flags);
        }

        std::string text(){
            return *m_text;
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
            m_context->postRedraw(0);
        }

        virtual bool mousePressEvent(MouseEvent const& m){
            if(bbox().contains(m.pos)){
                m_hovered = true;
                if(m.buttons & Qt::LeftButton)
                    m_pressed = true;
                // TODO: only when necessary
                m_context->postRedraw(0);
                return true;
            }
            return false;
        }

        virtual void mouseReleaseEvent(MouseEvent const& m){
            if(m_pressed && bbox().contains(m.pos))
                doAction();
            m_pressed = false;
            // TODO: only when necessary
            m_context->postRedraw(0);
        }

        virtual void mouseGoneEvent(){
            m_hovered = false;
            m_pressed = false;
            // TODO: only when necessary
            m_context->postRedraw(0);
        }

        virtual void doAction(){
            (*m_onclick)();
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
            : Menu(c), m_items(), m_filtered_items(), m_bbox(),
              m_filter_string(boost::make_shared<Text>(c, "")){
            typename item_map_t::const_iterator i;
            for(i = items.begin(); i != items.end(); i++){
                item_ptr ip = boost::make_shared<item_t>(c, *i);
                m_items.push_back(ip);
                m_bbox |= ip->bbox() + ip->m_pos;
            }
            typename std::list<item_ptr>::iterator j;
            for(j = m_items.begin(); j != m_items.end(); j++)
                (*j)->setSpan(m_bbox.min.x, m_bbox.max.x);
            _updateFilteredItems();
        }

        virtual ~ListMenu(){ }

        virtual void draw(drawtype_e::e flags){
            typename std::list<item_ptr>::const_iterator i;
            /* menus are above: */
            glTranslatef(0, 0, 0.1);
            glColor(Colour(1.0, 0.8));
            glBox(m_bbox);
            glColor(Colour(1.0, 0.5));
            if(m_filter_string->size()){
                glPushMatrix();                
                BBox f_bbox = m_filter_string->bbox();
                
                f_bbox.min.x = m_bbox.min.x;
                f_bbox.max.x = m_bbox.max.x;

                glTranslatef(m_filter_string->m_pos);            
                glBox(f_bbox);
                m_filter_string->draw(flags);
                glPopMatrix();
            }
            for(i = m_filtered_items.begin(); i != m_filtered_items.end(); i++){
                glPushMatrix();
                glTranslatef((*i)->m_pos);
                (*i)->draw(flags);
                glPopMatrix();
            }
        }
        
        virtual void mouseMoveEvent(MouseEvent const& m){
            typename std::list<item_ptr>::iterator i;
            std::set<item_ptr> now_hovered_items;
            for(i = m_filtered_items.begin(); i != m_filtered_items.end(); i++){
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

        virtual bool mousePressEvent(MouseEvent const& m){
            typename std::list<item_ptr>::iterator i;
            for(i = m_filtered_items.begin(); i != m_filtered_items.end(); i++){
                MouseEvent referred(m, *i);
                if((*i)->bbox().contains(referred.pos)){
                    if((*i)->mousePressEvent(referred))
                        m_pressed_items.insert(*i);
                }
            }
            return m_pressed_items.size() > 0;
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
        

        virtual bool keyPressEvent(KeyEvent const& event){
            std::string new_text = event.text().toStdString();        
            debug() << BashColour::White << "ListMenu: key press:" << new_text;
            switch(event.key()){
                case Qt::Key_Enter:
                case Qt::Key_Return:
                    debug() << "ListMenu: key enter";
                    if(m_filtered_items.size() == 1){
                        m_filtered_items.front()->doAction();
                        m_context->removeMenu(shared_from_this());
                        return true;
                    }
                    break;
                case Qt::Key_Delete:
                case Qt::Key_Backspace:
                    if(m_filter_string->size())
                        m_filter_string->erase(m_filter_string->end()-1);
                    break;
                case Qt::Key_Escape:
                    m_filter_string->clear();
                    break;
                default:
                    if(new_text.size())
                        *m_filter_string += new_text;
                    else
                        return false;
            }
            _updateFilteredItems();
            return true;
        }
        //virtual bool keyReleaseEvent(KeyEvent const&){ return false; }

        BBox bbox(){
            return m_bbox;
        }

    private:
        /* Update the displayed items and bounding box based on the filter
         * string
         */
        void _updateFilteredItems(){
            typename std::list<item_ptr>::iterator i;
            int y_pos = 0;
            int prev_height = 0;
            m_bbox = BBox();
            std::list<item_ptr> new_filtered_items;

            m_filter_string->updateBbox();
            m_filter_string->m_pos.y = -roundZ(m_filter_string->bbox().max.y);
            m_bbox |= m_filter_string->bbox() + m_filter_string->m_pos;

            y_pos -= 20;

            for(i = m_items.begin(); i != m_items.end(); i++){
                if(boost::icontains((*i)->text(), *m_filter_string)){
                    y_pos -= prev_height;
                    (*i)->m_pos.y = y_pos + roundZ((*i)->bbox().min.y);
                    prev_height = roundA((*i)->bbox().h());
                    m_bbox |= (*i)->bbox() + (*i)->m_pos;
                    new_filtered_items.push_back(*i);
                }
            }
            //if(new_filtered_items != m_filtered_items){
                m_filtered_items = new_filtered_items;
                m_context->postRedraw(0);
            //}
        }

        std::list<item_ptr> m_items;
        std::list<item_ptr> m_filtered_items;
        BBox m_bbox;

        boost::shared_ptr<Text> m_filter_string;

        std::set<item_ptr> m_hovered_items;
        std::set<item_ptr> m_pressed_items;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __LIST_MENU_RENDERABLE_H__

