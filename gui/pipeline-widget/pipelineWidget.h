/* Copyright 2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */

#ifndef __PIPELINE_WIDGET_H__
#define __PIPELINE_WIDGET_H__

#include <QGLWidget>
#include <QDesignerCustomWidgetInterface>

#include <set>
#include <map>

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "mouseEvent.h"
#include "container.h"
#include "pwTypes.h"

namespace cauv{

class Message;
class NodeAddedMessage;
class NodeParametersMessage;

namespace gui{
namespace pw{

class PipelineWidget: public QGLWidget,
                      public Container{
    Q_OBJECT
        // private typedefs:
        typedef std::set<renderable_ptr_t> renderable_set_t;
        typedef std::set<arc_ptr_t> arc_set_t;
        typedef std::map<node_id, node_ptr_t> node_map_t;
        typedef std::set<node_id> node_set_t;
        typedef std::map<node_id, imgnode_ptr_t> imgnode_map_t;
        typedef boost::recursive_mutex mutex_t;
        typedef boost::unique_lock<boost::recursive_mutex> lock_t;

        // friends:
        friend struct MouseEvent;

    public:
        PipelineWidget(QWidget *parent = 0);
        void initKeyBindings();
    
        QSize minimumSizeHint() const;        
        QSize sizeHint() const;
        
        std::string pipelineName() const;        
        
        void remove(menu_ptr_t);
        void remove(node_ptr_t);
        void add(renderable_ptr_t);
        void add(renderable_ptr_t, Point const& at);
        void addMenu(menu_ptr_t, Point const& at, bool pressed=false);
        void addNode(node_ptr_t);
        void addImgNode(imgnode_ptr_t);

        node_ptr_t node(node_id const&);
        std::vector<node_ptr_t> nodes() const;
        imgnode_ptr_t imgNode(node_id const&);
        
        // hmm
        typedef node_id const& ni_r;
        typedef std::string const& str_r;
        void addArc(ni_r src, str_r output,
                    node_id const& dst, std::string const& input);
        void addArc(renderable_ptr_t src,
                    node_id const& dst, std::string const& input);
        void addArc(node_id const& src, std::string const& output,
                    renderable_ptr_t dst);
        arc_ptr_t addArc(renderable_wkptr_t src, renderable_wkptr_t dst);

        void removeArc(node_id const& src, std::string const& output,
                       node_id const& dst, std::string const& input);
        void removeArc(renderable_ptr_t src,
                       node_id const& dst, std::string const& input);
        void removeArc(node_id const& src, std::string const& output,
                       renderable_ptr_t dst);
        void removeArc(renderable_ptr_t src, renderable_ptr_t dst);
        void sanitizeArcs();

        arc_ptr_t arcWithDestination(renderable_ptr_t dst);
        
        void send(boost::shared_ptr<Message>);
        
        node_ptr_t nodeAt(Point const& p) const;
        // implement Container:
        virtual Point referUp(Point const& p) const;
        virtual void postRedraw(float delay_secs);
        virtual void postMenu(menu_ptr_t m, Point const& top_level_position,
                              bool pressed=false);
		virtual void postText(const std::string &text, const std::string &font);
        virtual void removeMenu(menu_ptr_t);
        virtual void remove(renderable_ptr_t);
        
        void clear();
        void reload();

    public Q_SLOTS:
        // Causes GUI to discard all current state
        void setPipelineName(std::string const& name);
        void setPipelineName(const QString& name);

    Q_SIGNALS:
        void redrawPosted();
        void messageGenerated(boost::shared_ptr<Message>);
        void nameChanged(std::string const&);
    
    protected:
        void initializeGL();
        void paintGL();
        void resizeGL(int width, int height);
        void mousePressEvent(QMouseEvent *event);
        void mouseReleaseEvent(QMouseEvent *event);
        void mouseMoveEvent(QMouseEvent *event);

        void keyPressEvent(QKeyEvent *event);
        void keyReleaseEvent(QKeyEvent *event);

        void wheelEvent(QWheelEvent *event);
    
    private:
        void updateProjection();
        void projectionForPicking(int x, int y);
        void drawGrid();

        node_set_t parents(node_id n) const;
        node_set_t children(node_id n) const;

        // hotkey functions
        Point lastMousePosition() const;
        void duplicateNodeAtMouse();
        void removeNodeAtMouse();
        void testEditBoxMenu();
        void calcLayout();
        void changeNameMenu();

        
        Point m_win_centre;       // projected coordinates of the window
        double m_win_aspect;      // actually sqrt(x / y)
        double m_win_scale;       // width = scale*aspect, height = scale/aspect
        double m_scrolldelta;     // sum of scroll deltas
        double m_pixels_per_unit; // size on screen of a line of length 1

        const static int m_world_size = 100; // scale everything down by this much,
                                             // since canonical view volume is 1x1x1
        
        QPoint m_last_mouse_pos;
        
        ok::overlay_ptr_t m_overkey;
        menu_ptr_t m_menu;
        node_map_t m_nodes;
        imgnode_map_t m_imgnodes;
        arc_set_t m_arcs;

        renderable_set_t m_owning_mouse; // which renderables are involved in
                                         // the current mouse event
        renderable_set_t m_receiving_move; // renderables currently receiving
                                           // move events
        
        /* a great big mutex around everything since messages can cause all
         * sorts of havoc if they cause changes to the pipeline while drawing
         * is in progress, similarly for qt events:
         */
        mutable mutex_t m_lock;
        
        /* only allow one redraw to be queued at any time --- prevent queueing
         * of miiilllions of re-draws during a whole-graph update
         */
        mutable mutex_t m_redraw_posted_lock;
        bool m_redraw_posted;
        
        /* name of pipeline controlled by this widget: used to filter incoming
         * and construct outgoing messages
         */
        std::string m_pipeline_name;
};

} // namespace pw
} // namespace gui
} // namespace cauv

#endif // ndef __PIPELINE_WIDGET_H__
