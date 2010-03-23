#ifndef __PIPELINE_WIDGET_H__
#define __PIPELINE_WIDGET_H__

#include <QGLWidget>
#include <QDesignerCustomWidgetInterface>

class PipelineWidget: public QGLWidget{
    Q_OBJECT
    
    public:
        PipelineWidget(QWidget *parent = 0);
    
        QSize minimumSizeHint() const;        
        QSize sizeHint() const;
    
    protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void mousePressEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);
    
};


#endif // ndef __PIPELINE_WIDGET_H__
