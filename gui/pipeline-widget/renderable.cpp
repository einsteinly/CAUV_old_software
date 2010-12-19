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

