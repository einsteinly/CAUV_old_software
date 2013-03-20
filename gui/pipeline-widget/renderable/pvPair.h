/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __PV_PAIR_H__
#define __PV_PAIR_H__

#include <ios>

#include <boost/lexical_cast.hpp>

#include <utility/string.h>
#include <debug/debug.h>

#include "node.h"

namespace cauv{
namespace gui{
namespace pw{

class PVPairEditableBase: public Renderable{
    public:
        PVPairEditableBase(Node* n, bool editable=true)
            : Renderable(n), m_editable(editable){
        }
        virtual ~PVPairEditableBase(){ }

        bool editable(bool yn){
            return m_editable = yn;
        }

        bool editable() const {
            return m_editable;
        }

    protected:
        bool m_editable;
};

// Support editing BoundedFloat types:
template<typename charT, typename traitsT>
std::basic_ostream<charT,traitsT>& operator<<(std::basic_ostream<charT,traitsT>& os, BoundedFloat const& bf){
    os << bf.value;
    return os;
}

template<typename charT, typename traitsT>
std::basic_istream<charT,traitsT>& operator>>(std::basic_istream<charT,traitsT>& is, BoundedFloat& bf){
    is >> bf.value;
    if(bf.value < bf.min)
        bf.value = bf.min;
    if(bf.value > bf.max)
        bf.value = bf.max;
    return is;
}

template<typename value_T>
class PVPair: public PVPairEditableBase{
    public:
        typedef Node* node_ptr_t;
        PVPair(node_ptr_t n, std::string const& param, value_T const& value,
               bool editable=true)
            : PVPairEditableBase(n, editable), m_node(n), m_bbox(),
              m_min_value_bbox(0, -3, 13, 10),
              m_param(boost::make_shared<Text>(n, param)),
              m_equals(boost::make_shared<Text>(n, "=")),
              m_value(boost::make_shared<Text>(n, mkStr() << std::boolalpha << value)),
              m_old_value(value){
            updateBbox();
            m_sort_key = param;
        }
        virtual ~PVPair(){ }

        void draw(drawtype_e::e flags){
            glPushMatrix();
            glTranslatef(m_param->m_pos);
            m_param->draw(flags);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(m_equals->m_pos);
            m_equals->draw(flags);
            glPopMatrix();

            glPushMatrix();
            glTranslatef(m_value->m_pos);
            if(m_value->bbox().area()){
                if(m_editable)
                    m_value->colour(Colour(0));
                else
                    m_value->colour(Colour(0, 0.5));

                m_value->draw(flags);
            }else{
                glColor(Colour(0.8, 0.2, 0.2, 0.5));
                glBox(m_min_value_bbox);
            }
            glPopMatrix();
        }

        virtual bool mousePressEvent(MouseEvent const& e){
            MouseEvent referred(e, m_value);
            cauv::gui::BBox value_bbox = m_value->bbox();
            if(!value_bbox.area())
                value_bbox = m_min_value_bbox;
            if(value_bbox.contains(referred.pos) && m_editable){
                debug() << "got value hit";
                cauv::gui::BBox edit_box = value_bbox;
                edit_box.max.x += 40;
                edit_box.min.y -= 3;
                edit_box.max.y += 3;
                m_context->postMenu(
                    boost::make_shared<EditText<PVPair const&> >(
                        m_context, *m_value, edit_box, &onValueChanged, boost::ref(*this)
                    ),
                    m_context->referUp(m_pos + m_value->m_pos)
                );
                m_context->postRedraw(0);
                return true;
            }
            return false;
        }

        virtual cauv::gui::BBox bbox(){
            return m_bbox;
        }


    private:
        static void onValueChanged(PVPair const& pvp, std::string const& s){
            value_T v = pvp.m_old_value;
            std::istringstream is(s);
            is >> v;
            debug() << "PVPair: Edit done:" << s << "=" << std::boolalpha << v;
            if(!is){
                error() << "PVPair: Bad lexical cast reading " << s;
                return;
            }
            pvp.m_node->paramValueChanged(*pvp.m_param, v);
        }

        void updateBbox(){
            cauv::gui::BBox value_bbox = m_value->bbox();
            if(!value_bbox.area())
                value_bbox = m_min_value_bbox;
            
            m_bbox = m_param->bbox();
            m_equals->m_pos.x = m_bbox.max.x + 3 - m_equals->bbox().min.x;
            m_bbox |= m_equals->bbox() + m_equals->m_pos;
            m_value->m_pos.x = m_bbox.max.x + 3 - value_bbox.min.x;
            m_bbox |= value_bbox + m_value->m_pos;
        }

        node_ptr_t m_node;

        cauv::gui::BBox m_bbox;
        cauv::gui::BBox m_min_value_bbox;

        boost::shared_ptr<Text> m_param;
        boost::shared_ptr<Text> m_equals;
        boost::shared_ptr<Text> m_value;
        value_T m_old_value;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __PV_PAIR_H__
