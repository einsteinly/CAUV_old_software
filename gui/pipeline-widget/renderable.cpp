/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "renderable.h"
#include "pipelineWidget.h"

#include <utility/string.h>

using namespace cauv::gui::pw;

Renderable::Renderable(container_ptr_t c, cauv::gui::Point const& at)
    : m_pos(at), m_sort_key(toStr(this)), m_context(c){
}

cauv::gui::Point Renderable::topLevelPos() const{
    return m_context->referUp(m_pos);
}

