#include "renderable.h"
#include "pipelineWidget.h"

#include <common/cauv_utils.h>

using namespace pw;

Renderable::Renderable(container_ptr_t c, Point const& at)
    : m_pos(at), m_sort_key(to_string(this)), m_context(c){
}

Point Renderable::topLevelPos() const{
    return m_context->referUp(m_pos);
}

