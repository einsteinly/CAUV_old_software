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

#include "container.h"

#include <QtOpenGL>

#include <debug/cauv_debug.h>

#include "renderable.h"

using namespace cauv::pw;

renderable_ptr_t Container::pick(Point const& p){
    renderable_list_t::const_iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++)
        if((*i)->bbox().contains(p - (*i)->m_pos)){
            boost::shared_ptr<Container> cp;
            // eww, a bit nasty
            if((cp = boost::dynamic_pointer_cast<Container>(*i))){
                debug() << __func__ << "hit container";
                return cp->pick(p - (*i)->m_pos);
            }else{
                debug() << __func__ << "not a container";
                return *i;
            }
        }
    return renderable_ptr_t();
}

void Container::draw(drawtype_e::e flags){
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos);
        (*i)->draw(flags);
        glPopMatrix();
    }
}

BBox Container::bbox(){
    BBox r;
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++)
        r |= (*i)->bbox();
    return r;
}

