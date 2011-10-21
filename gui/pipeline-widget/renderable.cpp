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

#include "renderable.h"
#include "pipelineWidget.h"

#include <utility/string.h>

using namespace cauv::pw;

Renderable::Renderable(container_ptr_t c, Point const& at)
    : m_pos(at), m_sort_key(toStr(this)), m_context(c){
}

Point Renderable::topLevelPos() const{
    return m_context->referUp(m_pos);
}

