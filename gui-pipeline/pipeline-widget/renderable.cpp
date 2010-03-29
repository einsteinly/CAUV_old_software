#include "renderable.h"
#include "pipelineWidget.h"

Renderable::Renderable(PipelineWidget& p, Point const& at)
    : m_pos(at), m_parent(p){
}


