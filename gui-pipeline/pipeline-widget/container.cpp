#include "container.h"

#include <QtOpenGL>

#include "renderable.h"

void Container::draw(bool picking){
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos);
        (*i)->draw(picking);
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

