#include "pipelineWidget.h"

PipelineWidget::PipelineWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent){
    // TODO: more appropriate QGLFormat?
}

QSize PipelineWidget::minimumSizeHint() const{
    return QSize(200, 200);
}

QSize PipelineWidget::sizeHint() const{
    return QSize(800, 800);
}

void PipelineWidget::initializeGL(){
    qglClearColor(QColor(20, 20, 20, 255));
    glShadeModel(GL_SMOOTH);
    glEnable(GL_MULTISAMPLE);
}

void PipelineWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
}

void PipelineWidget::resizeGL(int width, int height){
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
}

void PipelineWidget::mousePressEvent(QMouseEvent *event){
}

void PipelineWidget::mouseMoveEvent(QMouseEvent *event){
}
