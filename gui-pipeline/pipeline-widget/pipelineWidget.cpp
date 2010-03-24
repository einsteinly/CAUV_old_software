#include "pipelineWidget.h"

#include <QtGui>

#include <cmath>

#include <common/debug.h>

#include "renderable/box.h"

PipelineWidget::PipelineWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      m_win_centre_x(0), m_win_centre_y(0),
      m_win_aspect(1), m_win_scale(10),
      m_pixels_per_unit(10),
      m_last_mouse_pos(){
    // TODO: more appropriate QGLFormat?
    
    setMouseTracking(true);
    m_renderables.insert(boost::make_shared<Box>(boost::ref(*this), 1, 1));
}

QSize PipelineWidget::minimumSizeHint() const{
    return QSize(200, 200);
}

QSize PipelineWidget::sizeHint() const{
    return QSize(800, 800);
}

void PipelineWidget::initializeGL(){
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_CULL_FACE); 
    
    glShadeModel(GL_SMOOTH);
    glCullFace(GL_BACK);
    glClearColor(0.1, 0.1, 0.1, 1.0);
    glClearDepth(100.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PipelineWidget::paintGL(){
    updateProjection();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    // scale for world size 'out of screen' is positive z
    glScalef(1.0f/m_world_size, 1.0f/m_world_size, 1.0f);
    drawGrid();
    
    // debug stuff:
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 0.5);
    glVertex2f(-1.0f, 0.0f);
    glVertex2f(1.0f, 0.0f);
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glVertex2f(0.0f, -1.0f);
    glVertex2f(0.0f, 1.0f);

    glColor4f(1.0, 0.0, 0.0, 0.5);
    glVertex2f(-2.0f, -1.75f);
    glVertex2f(-1.5f, -1.75f);
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glVertex2f(-1.75f, -2.0f);
    glVertex2f(-1.75f, -1.5f);
    glEnd();

    // draw everything!
    renderable_set_t::iterator i;
    for(i = m_renderables.begin(); i != m_renderables.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos_x, (*i)->m_pos_y, 0);
        (*i)->draw();
        glPopMatrix();
    }
}

void PipelineWidget::resizeGL(int width, int height){
    m_win_aspect = sqrt(double(width) / height);
    m_win_scale = sqrt(width*height) / m_pixels_per_unit;
    debug() << __func__
            << "width=" << width << "height=" << height
            << "aspect=" << m_win_aspect << "scale=" << m_win_scale;
    
    glViewport(0, 0, width, height);
    
    updateProjection();
}

void PipelineWidget::mousePressEvent(QMouseEvent *event){
	GLuint hits = 0;
    GLuint n = 0;
    GLuint pick_buffer[100] = {0};
    GLuint *p;
    GLuint *item;
    renderable_set_t::iterator i;

    typedef std::map<GLuint, renderable_ptr_t> name_map_t;
    name_map_t name_map;

	glSelectBuffer(sizeof(pick_buffer)/sizeof(GLuint), pick_buffer); 
	glRenderMode(GL_SELECT);

	glInitNames(); 
	glPushName(GLuint(-1));
    
    projectionForPicking(event->x(), event->y());
    glLoadIdentity();
    glScalef(1.0f/m_world_size, 1.0f/m_world_size, 1.0f);

    for(i = m_renderables.begin(); i != m_renderables.end(); i++, n++){
        if((*i)->acceptsMouseEvents()){
			glLoadName(n);
            name_map[n] = *i;
            glPushMatrix();
            glTranslatef((*i)->m_pos_x, (*i)->m_pos_y, 0);
            (*i)->draw();
            glPopMatrix();
        }
    }
    glPopName();
    glFlush();
    hits = glRenderMode(GL_RENDER);
    debug() << "rendered" << n << "items for pick," << hits << "hit";

    p = pick_buffer;
    for(unsigned i = 0; i < hits; i++, p += (*p) + 3){
		item = p+3;
		for(unsigned j = 0; j < *p; j++, item++){
			name_map_t::const_iterator k = name_map.find(*item);
            if(k == name_map.end()){
                error() << "gl name" << *item << "does not correspond to renderable";
            }else{
                debug() << "sending mouse press event to" << k->second;
                k->second->mousePressEvent(MouseEvent(event, k->second, *this));
                m_owning_mouse.insert(k->second);
            }
		}
	}


    m_last_mouse_pos = event->pos();
}

void PipelineWidget::mouseReleaseEvent(QMouseEvent *event){
    renderable_set_t::iterator i;
    for(i = m_owning_mouse.begin(); i != m_owning_mouse.end(); i++){
       debug() << "sending mouse release event to" << *i;    
       (*i)->mouseReleaseEvent(MouseEvent(event, *i, *this));
    }
    if(!event->buttons()){
        m_owning_mouse.clear();
    }
}

void PipelineWidget::mouseMoveEvent(QMouseEvent *event){
    if(!m_owning_mouse.size()){
        int win_dx = event->x() - m_last_mouse_pos.x();
        int win_dy = event->y() - m_last_mouse_pos.y();
        if(event->buttons() & Qt::LeftButton){
            double dx = win_dx / (m_pixels_per_unit*m_world_size);
            double dy = -win_dy / (m_pixels_per_unit*m_world_size); // NB -
            m_win_centre_x += dx;
            m_win_centre_y += dy;
            updateGL();
        }
        // else zoom, (TODO)
    }else{
        renderable_set_t::iterator i;
        for(i = m_owning_mouse.begin(); i != m_owning_mouse.end(); i++){
            debug() << "sending mouse move event to" << *i;
            (*i)->mouseMoveEvent(MouseEvent(event, *i, *this));
        }
    }
    m_last_mouse_pos = event->pos();
}

void PipelineWidget::updateProjection(){
    double w = (m_win_scale * m_win_aspect) / m_world_size;
    double h = (m_win_scale / m_win_aspect) / m_world_size;
    debug() << __func__
            << "w=" << w << "h=" << h
            << "aspect=" << m_win_aspect << "scale=" << m_win_scale
            << "res=" << m_pixels_per_unit
            << "x=" << m_win_centre_x << "y=" << m_win_centre_y;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glTranslatef(0.375 * w / width(), 0.375 * h / height(), 0);
    glOrtho(-w/2, w/2, -h/2, h/2, -100, 100);
    glTranslatef(m_win_centre_x, m_win_centre_y, 0);
    glMatrixMode(GL_MODELVIEW);
}

void PipelineWidget::projectionForPicking(int mouse_win_x, int mouse_win_y){
    double w = (m_win_scale * m_win_aspect) / m_world_size;
    double h = (m_win_scale / m_win_aspect) / m_world_size;
    debug() << __func__
            << "w=" << w << "h=" << h
            << "aspect=" << m_win_aspect << "scale=" << m_win_scale
            << "res=" << m_pixels_per_unit
            << "x=" << m_win_centre_x << "y=" << m_win_centre_y;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLint viewport[4] = {0};
    glGetIntegerv(GL_VIEWPORT, viewport);
    gluPickMatrix((GLdouble)mouse_win_x, (GLdouble)(viewport[3]-mouse_win_y), 
                  1, 1, viewport);
    
    //glTranslatef(0.375 * w / width(), 0.375 * h / height(), 0);
    glOrtho(-w/2, w/2, -h/2, h/2, -100, 100);
    glTranslatef(m_win_centre_x, m_win_centre_y, 0);
    glMatrixMode(GL_MODELVIEW);}

static int roundToZ(double d){
    if (d < 0.0)
        return int(std::ceil(d));
    else
        return int(std::floor(d));
}

void PipelineWidget::drawGrid(){
    // only draw grid that will be visible:
    const double grid_major_spacing = 10;
    const double grid_minor_spacing = 1;
    
    // projected window coordinates:
    const double divisor = 2 * m_pixels_per_unit;
    const double min_x = -m_win_centre_x * m_world_size - width()  / divisor;
    const double min_y = -m_win_centre_y * m_world_size - height() / divisor;
    const double max_x = -m_win_centre_x * m_world_size + width()  / divisor;
    const double max_y = -m_win_centre_y * m_world_size + height() / divisor;
    
    const int min_grid_minor_x = roundToZ(min_x / grid_minor_spacing);
    const int min_grid_minor_y = roundToZ(min_y / grid_minor_spacing);
    const int max_grid_minor_x = roundToZ(max_x / grid_minor_spacing);
    const int max_grid_minor_y = roundToZ(max_y / grid_minor_spacing);
    
    const int min_grid_major_x = roundToZ(min_x / grid_major_spacing);
    const int min_grid_major_y = roundToZ(min_y / grid_major_spacing);
    const int max_grid_major_x = roundToZ(max_x / grid_major_spacing);
    const int max_grid_major_y = roundToZ(max_y / grid_major_spacing);
    
    debug(-1) << "min_x=" << min_x << "max_x=" << max_x
            << "min_y=" << min_y << "max_y=" << max_y << "\n\t"
            << "min_grid_minor_x=" << min_grid_minor_x
            << "max_grid_minor_x=" << max_grid_minor_x << "\n\t"
            << "min_grid_minor_y=" << min_grid_minor_y
            << "max_grid_minor_y=" << max_grid_minor_y << "\n\t"
            << "min_grid_major_x=" << min_grid_major_x
            << "max_grid_major_x=" << max_grid_major_x << "\n\t"
            << "min_grid_major_y=" << min_grid_major_y
            << "max_grid_major_y=" << max_grid_major_y;

    glColor4f(0.12, 0.1, 0.1, 0.2);
    glBegin(GL_LINES);
    for(int i = min_grid_minor_y; i <= max_grid_minor_y; i++){
        glVertex3f(min_x, i*grid_minor_spacing, -0.2);
        glVertex3f(max_x, i*grid_minor_spacing, -0.2);
    }

    glColor4f(0.1, 0.12, 0.1, 0.2);
    for(int i = min_grid_minor_x; i <= max_grid_minor_x; i++){
        glVertex3f(i*grid_minor_spacing, min_y, -0.2);
        glVertex3f(i*grid_minor_spacing, max_y, -0.2);
    }
    
    glColor4f(0.24, 0.2, 0.2, 0.3);
    for(int i = min_grid_major_y; i <= max_grid_major_y; i++){
        glVertex3f(min_x, i*grid_major_spacing, -0.1);
        glVertex3f(max_x, i*grid_major_spacing, -0.1);
    }

    glColor4f(0.2, 0.24, 0.2, 0.3);
    for(int i = min_grid_major_x; i <= max_grid_major_x; i++){
        glVertex3f(i*grid_major_spacing, min_y, -0.1);
        glVertex3f(i*grid_major_spacing, max_y, -0.1);
    }
    glEnd();
}

