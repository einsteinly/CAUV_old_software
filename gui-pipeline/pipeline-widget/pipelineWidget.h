#ifndef __PIPELINE_WIDGET_H__
#define __PIPELINE_WIDGET_H__

#include <QGLWidget>
#include <QDesignerCustomWidgetInterface>

#include <set>

#include <boost/shared_ptr.hpp>

#include "renderable.h"

class PipelineWidget: public QGLWidget{
    Q_OBJECT
    // private typedefs:
        typedef boost::shared_ptr<Renderable> renderable_ptr_t;
        typedef std::set<renderable_ptr_t> renderable_set_t;
    // friends:
        friend MouseEvent::MouseEvent(QMouseEvent*,
                                      boost::shared_ptr<Renderable>,
                                      PipelineWidget const&);    
    public:
        PipelineWidget(QWidget *parent = 0);
    
        QSize minimumSizeHint() const;        
        QSize sizeHint() const;
        
        void remove(Renderable const*);
        void add(boost::shared_ptr<Renderable>);
    
    protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void mousePressEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);
    
    private:
        void updateProjection();
        void projectionForPicking(int x, int y);
        void drawGrid();
        
        double m_win_centre_x;    // projected coordinates of the window
        double m_win_centre_y;    //
        double m_win_aspect;      // actually sqrt(x / y)
        double m_win_scale;       // width = scale*aspect, height = scale/aspect
        double m_pixels_per_unit; // size on screen of a line of length 1

        const static int m_world_size = 100; // scale everything down by this much,
                                             // since canonical view volume is 1x1x1
        
        QPoint m_last_mouse_pos;
        
        renderable_set_t m_renderables;
        renderable_set_t m_owning_mouse; // which renderables are involved in
                                         // the current mouse event
};


#endif // ndef __PIPELINE_WIDGET_H__
