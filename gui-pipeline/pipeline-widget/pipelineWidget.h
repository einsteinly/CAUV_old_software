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
    
    private:
        void updateProjection();
        
        double m_win_centre_x;    // projected coordinates of the window
        double m_win_centre_y;    //
        double m_win_aspect;      // actually sqrt(x / y)
        double m_win_scale;       // width = scale*aspect, height = scale/aspect
        double m_pixels_per_unit; // size on screen of a line of length 1
        
        QPoint m_last_mouse_pos;
};


#endif // ndef __PIPELINE_WIDGET_H__
