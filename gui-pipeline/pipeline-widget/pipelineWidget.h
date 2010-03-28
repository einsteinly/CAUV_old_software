#ifndef __PIPELINE_WIDGET_H__
#define __PIPELINE_WIDGET_H__

#include <QGLWidget>
#include <QDesignerCustomWidgetInterface>

#include <set>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "mouseEvent.h"

class PipelineGuiCauvNode;
class Message;
class Renderable;
class Node;
class Menu;

class PipelineWidget: public QGLWidget{
    Q_OBJECT
    // public typedefs:
    public:
        typedef boost::shared_ptr<Renderable> renderable_ptr_t;
        typedef boost::shared_ptr<Node> node_ptr_t;
        typedef boost::shared_ptr<Menu> menu_ptr_t;
        // TODO: this should really be synchronised with pipelineTypes.h! (need
        // a pipeline namespace to do that without confusion about Nodes though)
        typedef int32_t node_id;

    private:
    // private typedefs:
        typedef std::set<renderable_ptr_t> renderable_set_t;
        typedef std::map<node_id, node_ptr_t> node_map_t;

    // friends:
        friend MouseEvent::MouseEvent(QMouseEvent*,
                                      renderable_ptr_t,
                                      PipelineWidget const&);
        friend MouseEvent::MouseEvent(renderable_ptr_t,
                                      PipelineWidget const&);  
    public:
        PipelineWidget(QWidget *parent = 0);
    
        QSize minimumSizeHint() const;        
        QSize sizeHint() const;
        
        void remove(renderable_ptr_t);
        void remove(menu_ptr_t);
        void remove(node_ptr_t);
        void add(renderable_ptr_t);
        void add(renderable_ptr_t, double x, double y);
        void addMenu(menu_ptr_t, double x, double y);
        void addNode(node_ptr_t);

        node_ptr_t node(node_id const&);
        
        void setCauvNode(boost::shared_ptr<PipelineGuiCauvNode>);
        void sendMessage(boost::shared_ptr<Message>);
    
    protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void mousePressEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);

        void keyPressEvent(QKeyEvent* event);
        void keyReleaseEvent(QKeyEvent* event);
    
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
        menu_ptr_t m_menu;
        node_map_t m_nodes;

        renderable_set_t m_owning_mouse; // which renderables are involved in
                                         // the current mouse event
        renderable_set_t m_receiving_move; // renderables currently receiving
                                           // move events
        
        boost::shared_ptr<PipelineGuiCauvNode> m_cauv_node;
        boost::thread m_cauv_node_thread;
};


#endif // ndef __PIPELINE_WIDGET_H__
