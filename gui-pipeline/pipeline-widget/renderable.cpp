#include "renderable.h"
#include "pipelineWidget.h"

Renderable::Renderable(PipelineWidget& p, double x, double y)
    : m_pos_x(x), m_pos_y(y), m_parent(p){
}


