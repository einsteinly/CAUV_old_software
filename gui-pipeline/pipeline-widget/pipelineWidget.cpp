#include "pipelineWidget.h"

#include <QtGui>

#include <cmath>

#include <common/debug.h>

PipelineWidget::PipelineWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      m_win_centre_x(0), m_win_centre_y(0),
      m_win_aspect(1), m_win_scale(10),
      m_pixels_per_unit(10),
      m_last_mouse_pos(){
    // TODO: more appropriate QGLFormat?
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
    
    glColor4f(1.0, 0.0, 0.0, 0.5);
    glBegin(GL_LINES);
    glVertex2f(-1.0f, 0.0f);
    glVertex2f(1.0f, 0.0f);
    glEnd();
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glBegin(GL_LINES);
    glVertex2f(0.0f, -1.0f);
    glVertex2f(0.0f, 1.0f);
    glEnd();
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
    m_last_mouse_pos = event->pos();
}

void PipelineWidget::mouseMoveEvent(QMouseEvent *event){
    int win_dx = event->x() - m_last_mouse_pos.x();
    int win_dy = event->y() - m_last_mouse_pos.y();
    if(event->buttons() & Qt::LeftButton){
        double dx = win_dx / m_pixels_per_unit;
        double dy = -win_dy / m_pixels_per_unit; // win coordinates upside-down
        m_win_centre_x += dx;
        m_win_centre_y += dy;
        updateGL();
    }
    m_last_mouse_pos = event->pos();
}

void PipelineWidget::updateProjection(){
    double w = m_win_scale * m_win_aspect;
    double h = m_win_scale / m_win_aspect;
    debug() << __func__
    << "w=" << w << "h=" << h
    << "aspect=" << m_win_aspect << "scale=" << m_win_scale
    << "res=" << m_pixels_per_unit
    << "x=" << m_win_centre_x << "y=" << m_win_centre_y;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-w/2, w/2, -h/2, h/2, -100, 100);
    glTranslatef(m_win_centre_x, m_win_centre_y, 0);
    glMatrixMode(GL_MODELVIEW);
}
