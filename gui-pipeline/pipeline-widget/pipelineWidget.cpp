#include "pipelineWidget.h"

#include <QtGui>

#include <boost/thread.hpp>
#include <boost/ref.hpp>

#include <cmath>

#include <common/debug.h>
#include <common/cauv_node.h>
#include <common/messages.h>
#include <common/bash_cout.h>

#include "util.h"
#include "renderable.h"
#include "buildMenus.h"
#include "renderable/box.h"
#include "renderable/node.h"
#include "renderable/menu.h"
#include "renderable/editText.h"
#include "renderable/arc.h"


namespace pw{

bool operator==(arc_ptr_t a, arc_ptr_t b){
    return ((a->m_src.lock() == b->m_src.lock() && a->m_dst.lock() == b->m_dst.lock()) ||
            (a->m_dst.lock() == b->m_src.lock() && a->m_src.lock() == b->m_dst.lock()));
}

class PipelineGuiMsgObs: public MessageObserver{
    public:
        PipelineGuiMsgObs(PipelineWidget *p)
            : m_widget(p){
        }

        virtual void onNodeAddedMessage(NodeAddedMessage_ptr m){
            debug() << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
            if(m->nodeType() != NodeType::Invalid)
                m_widget->addNode(boost::make_shared<Node>(m_widget, m_widget, m));
        }

        virtual void onNodeParametersMessage(NodeParametersMessage_ptr m){
            debug() << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
            boost::shared_ptr<Node> np = m_widget->node(m->nodeId());
            if(np)
                np->setParams(m);
        }

        virtual void onArcAddedMessage(ArcAddedMessage_ptr m){
            debug() << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
            m_widget->addArc(m->from().node, m->from().output,
                             m->to().node, m->to().input);
        }

        virtual void onArcRemovedMessage(ArcRemovedMessage_ptr m){
            debug() << BashColour::Green << "PiplineGuiMsgObs:" << __func__ << *m;
            m_widget->removeArc(m->from().node, m->from().output,
                                m->to().node, m->to().input);
        }

    private:
        PipelineWidget *m_widget;
};

class PipelineGuiCauvNode: public CauvNode{
    public:
        PipelineGuiCauvNode(PipelineWidget *p)
            : CauvNode("pipe-gui"), m_widget(p){
            debug() << "PGCN constructed";
        }

        void onRun(){
            debug() << "PGCN::onRun()";
            mailbox()->joinGroup("pl_gui");
            mailboxMonitor()->addObserver(
                boost::make_shared<PipelineGuiMsgObs>(m_widget)
            );
            #if 0
            mailboxMonitor()->addObserver(
                boost::make_shared<DebugMessageObserver>()
            );
            #endif
        }

        void sendMessage(boost::shared_ptr<Message> m){
            // TODO: need mutex protection? think probably not
            debug() << "PGCN::sendMessage" << *m;
            mailbox()->sendMessage(m, SAFE_MESS);
        }
    private:
        PipelineWidget *m_widget;
};

} // namespace pw

using namespace pw;

// creating threads taking parameters (especially in a ctor-initializer) is a
// little tricky, using an intermediate function smooths the ride a bit:
void spawnPGCN(PipelineWidget *p){
    boost::shared_ptr<PipelineGuiCauvNode> pgcn =
        boost::make_shared<PipelineGuiCauvNode>(p);
    p->setCauvNode(pgcn);
    pgcn->run();
    warning() << __func__ << "run() finished";
}

PipelineWidget::PipelineWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      m_win_centre(), m_win_aspect(1), m_win_scale(10),
      m_pixels_per_unit(1),
      m_last_mouse_pos(),
      m_cauv_node(),
      m_cauv_node_thread(boost::thread(spawnPGCN, this)){
    // TODO: more appropriate QGLFormat?
    
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);

    #if 0
    m_contents.push_back(boost::make_shared<Box>(this, 20, 20));
    #endif
}

QSize PipelineWidget::minimumSizeHint() const{
    return QSize(200, 200);
}

QSize PipelineWidget::sizeHint() const{
    return QSize(800, 800);
}

void PipelineWidget::remove(renderable_ptr_t p){
    // TODO: really need m_contents to be a set in which iterators remain
    // stable on erasing, so that this doesn't involve a search:
    renderable_list_t::iterator i, j;
    for(i = m_contents.begin(); i != m_contents.end(); i=j){
        j = i; j++;
        if(*i == p)
            m_contents.erase(i);
    }
    arc_ptr_t a;
    if(a = boost::dynamic_pointer_cast<Arc>(p))
        m_arcs.erase(a);
    this->updateGL();
}

void PipelineWidget::remove(node_ptr_t n){
    m_nodes.erase(n->id());
    remove(renderable_ptr_t(n));
}

void PipelineWidget::remove(menu_ptr_t p){
    if(m_menu == p)
        m_menu.reset();
    remove(renderable_ptr_t(p));
}

void PipelineWidget::add(renderable_ptr_t r){
    // TODO: sensible layout hint
    static Point add_position_delta = Point();
    if(add_position_delta.sxx() > 200)
        add_position_delta = Point();
    else
        add_position_delta += Point(10, -5);
    MouseEvent last_mouse(*this);
    add(r, last_mouse.pos + add_position_delta);
}

void PipelineWidget::add(renderable_ptr_t r, Point const& at){
    r->m_pos = at;
    m_contents.push_back(r);
    this->updateGL();
}

void PipelineWidget::addMenu(menu_ptr_t r,  Point const& at, bool pressed){
    debug() << "addMenu:" << r << at << pressed;
    if(m_menu)
        remove(m_menu);
    if(pressed){
        m_receiving_move.insert(r);
        m_owning_mouse.insert(r);
    }
    m_menu = r;
    add(r, at);
}

void PipelineWidget::addNode(node_ptr_t r){
    m_nodes[r->id()] = r;
    add(r);
}

node_ptr_t PipelineWidget::node(node_id const& n){
    node_map_t::const_iterator i = m_nodes.find(n);
    if(i != m_nodes.end())
        return m_nodes[n];
    warning() << "unknown node renderable:" << n;
    return node_ptr_t();
}


void PipelineWidget::addArc(node_id const& src, std::string const& output,
                            node_id const& dst, std::string const& input){
    node_ptr_t s = node(src);
    node_ptr_t d = node(dst);
    if(!s || !d)
        return;
    renderable_ptr_t s_no = s->outSocket(output);
    renderable_ptr_t d_ni = d->inSocket(input);
    if(!s_no || !d_ni)
        return;
    addArc(s_no, d_ni);
}

void PipelineWidget::removeArc(node_id const& src, std::string const& output,
                               node_id const& dst, std::string const& input){
    node_ptr_t s = node(src);
    node_ptr_t d = node(dst);
    if(!s || !d)
        return; 
    renderable_ptr_t s_no = s->outSocket(output);
    renderable_ptr_t d_ni = d->inSocket(input);
    if(!s_no || !d_ni)
        return; 
    arc_ptr_t a = boost::make_shared<Arc>(this, s_no, d_ni);
    for(arc_set_t::const_iterator i = m_arcs.begin(); i != m_arcs.end(); i++)
        if(*i == a){
            remove(*i);
            return;
        }
    warning() << __func__ << "no such arc" << src << output  << "->" << dst << input;
}

void PipelineWidget::addArc(renderable_ptr_t src,
                            node_id const& dst, std::string const& input){
    node_ptr_t d = node(dst);
    if(!d) return;
    renderable_ptr_t d_ni = d->inSocket(input); 
    if(!d_ni) return;
    addArc(src, d_ni);
}

void PipelineWidget::addArc(node_id const& src, std::string const& output,
                            renderable_ptr_t dst){
    node_ptr_t s = node(src);
    if(!s) return;
    renderable_ptr_t s_no = s->outSocket(output);
    if(!s_no) return;
    addArc(s_no, dst);
}

arc_ptr_t PipelineWidget::addArc(renderable_wkptr_t src, renderable_wkptr_t dst){
    arc_ptr_t a = boost::make_shared<Arc>(this, src, dst);
    for(arc_set_t::const_iterator i = m_arcs.begin(); i != m_arcs.end(); i++)
        if(*i == a){
            warning() << "duplicate arc will be ignored";
            return *i;
        }
    m_arcs.insert(a);
    add(a, Point());
    return a;
}

void PipelineWidget::setCauvNode(boost::shared_ptr<PipelineGuiCauvNode> c){
    if(m_cauv_node)
        warning() << "PipelineWidget::setCauvNode already set";
    m_cauv_node = c;
}

void PipelineWidget::sendMessage(boost::shared_ptr<Message> m){
    if(m_cauv_node)
        m_cauv_node->sendMessage(m);
    else
        error() << "PipelineWidget::sendMessage no associated cauv node";
}


Point PipelineWidget::referUp(Point const& p) const{
    // nowhere up to refer to: just return the same point
    return p;
}

void PipelineWidget::postRedraw(){
    this->updateGL();
}

void PipelineWidget::postMenu(menu_ptr_t m, Point const& p, bool r) {
    addMenu(m, p, r);
}

void PipelineWidget::removeMenu(menu_ptr_t r){
    remove(r);
}


void PipelineWidget::initializeGL(){
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_CULL_FACE); 
    glEnable(GL_BLEND);

    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthFunc(GL_LEQUAL);
    
    glShadeModel(GL_SMOOTH);
    glCullFace(GL_BACK);
    glClearColor(0, 0, 0, 1.0);
    glClearDepth(100.0);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

void PipelineWidget::paintGL(){
    updateProjection();
    
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    // scale for world size 'out of screen' is positive z
    glScalef(1.0f/m_world_size, 1.0f/m_world_size, 1.0f);
    glTranslatef(0.5/m_pixels_per_unit, 0.5/m_pixels_per_unit, 0);
    drawGrid();
    
    #if 0
    // debug stuff:
    glBegin(GL_LINES);
    glColor4f(1.0, 0.0, 0.0, 0.5);
    glVertex2f(-10.0f, 0.0f);
    glVertex2f(10.0f, 0.0f);
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glVertex2f(0.0f, -10.0f);
    glVertex2f(0.0f, 10.0f);

    glColor4f(1.0, 0.0, 0.0, 0.5);
    glVertex2f(-20.0f, -17.5f);
    glVertex2f(-15.0f, -17.5f);
    
    glColor4f(0.0, 1.0, 0.0, 0.5);
    glVertex2f(-17.5f, -20.0f);
    glVertex2f(-17.5f, -15.0f);
    glEnd();
    #endif

    // draw everything!
    renderable_list_t::iterator i;
    for(i = m_contents.begin(); i != m_contents.end(); i++){
        glPushMatrix();
        glTranslatef((*i)->m_pos);
        (*i)->draw(false);
        glPopMatrix();
    }
}

void PipelineWidget::resizeGL(int width, int height){
    m_win_aspect = sqrt(double(width) / height);
    m_win_scale = sqrt(width*height) / m_pixels_per_unit;
    debug(-1) << __func__
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
    renderable_list_t::iterator i;

    typedef std::map<GLuint, renderable_ptr_t> name_map_t;
    name_map_t name_map;

	glSelectBuffer(sizeof(pick_buffer)/sizeof(GLuint), pick_buffer); 
	glRenderMode(GL_SELECT);

	glInitNames(); 
	glPushName(GLuint(-1));
    
    projectionForPicking(event->x(), event->y());
    glLoadIdentity();
    glScalef(1.0f/m_world_size, 1.0f/m_world_size, 1.0f);
    glTranslatef(0.5/m_pixels_per_unit, 0.5/m_pixels_per_unit, 0);    

    for(i = m_contents.begin(); i != m_contents.end(); i++, n++){
        if((*i)->acceptsMouseEvents()){
			glLoadName(n);
            name_map[n] = *i;
            glPushMatrix();
            glTranslatef((*i)->m_pos);
            (*i)->draw(true);
            glPopMatrix();
        }
    }
    glPopName();
    glFlush();
    hits = glRenderMode(GL_RENDER);
    debug() << "rendered" << n << "items for pick," << hits << "hit";

    bool need_redraw = false;

    if(m_menu){
        remove(m_menu);
        need_redraw = true;
    }

    p = pick_buffer;
    for(unsigned i = 0; i < hits; i++, p += (*p) + 3){
		item = p+3;
		for(unsigned j = 0; j < *p; j++, item++){
			name_map_t::const_iterator k = name_map.find(*item);
            if(k == name_map.end()){
                error() << "gl name" << *item << "does not correspond to renderable";
            }else{
                debug(-1) << "sending mouse press event to" << k->second;
                k->second->mousePressEvent(MouseEvent(event, k->second, *this));
                m_owning_mouse.insert(k->second);
            }
		}
	}

    if(event->buttons() & Qt::RightButton){
        MouseEvent proxy(event, *this);
        addMenu(buildAddNodeMenu(this), proxy.pos);
        need_redraw = true; 
    }
    
    m_last_mouse_pos = event->pos();

    if(need_redraw)
        this->updateGL();
}

void tdf(int, std::string const& s){
    debug() << "Edit done:" << s;
}

void PipelineWidget::keyPressEvent(QKeyEvent* event){
    BBox temp_bbox(0, -6, 80, 10);
    if(m_menu){
        if(!m_menu->keyPressEvent(event))
            QWidget::keyPressEvent(event);
    }else{
        MouseEvent proxy(*this);
        // TODO: hotkeys
        switch(event->key()){
            case Qt::Key_Space:
            case Qt::Key_A:
                addMenu(buildAddNodeMenu(this), proxy.pos);
                break;
            case Qt::Key_E:
                addMenu(boost::make_shared< EditText<int> >(
                            this, std::string("edit here"), temp_bbox, tdf, 0
                        ), proxy.pos);
                break;
            default:
                QWidget::keyPressEvent(event);
                break;
        }
    }
}

void PipelineWidget::keyReleaseEvent(QKeyEvent* event){
    if(!m_menu || !m_menu->keyReleaseEvent(event)){
        QWidget::keyReleaseEvent(event);
    }
}

void PipelineWidget::mouseReleaseEvent(QMouseEvent *event){
    renderable_set_t::iterator i;
    for(i = m_owning_mouse.begin(); i != m_owning_mouse.end(); i++){
       debug(-1) << "sending mouse release event to" << *i;    
       (*i)->mouseReleaseEvent(MouseEvent(event, *i, *this));
    }
    if(!event->buttons()){
        m_owning_mouse.clear();
    }
}

void PipelineWidget::mouseMoveEvent(QMouseEvent *event){
    renderable_set_t::iterator i;
    renderable_list_t::iterator j;
    if(!m_owning_mouse.size()){
        int win_dx = event->x() - m_last_mouse_pos.x();
        int win_dy = event->y() - m_last_mouse_pos.y();
        if(event->buttons() & Qt::LeftButton){
            Point d(win_dx / m_pixels_per_unit,
                    -win_dy / m_pixels_per_unit); // NB -
            m_win_centre += d;
            this->updateGL();
        }
        // else zoom, (TODO)
    }else{
        for(i = m_owning_mouse.begin(); i != m_owning_mouse.end(); i++){
            debug(-1) << "sending mouse move event to" << *i;
            (*i)->mouseMoveEvent(MouseEvent(event, *i, *this));
        }
    }
    renderable_set_t now_receiving_move = m_owning_mouse;
    // in all cases send the event to things that always track mouse movement:
    for(j = m_contents.begin(); j != m_contents.end(); j++)
        if((*j)->tracksMouse() && !m_owning_mouse.count(*j)){
            MouseEvent m = MouseEvent(event, *j, *this);
            if((*j)->bbox().contains(m.pos)){
                (*j)->mouseMoveEvent(m);
                now_receiving_move.insert(*j);
            }
        }
    // and send mouseGoneEvents to things that the mouse has left
    for(i = m_receiving_move.begin(); i != m_receiving_move.end(); i++)
        if(!now_receiving_move.count(*i))
            (*i)->mouseGoneEvent();
    m_receiving_move = now_receiving_move;

    m_last_mouse_pos = event->pos();
}

void PipelineWidget::updateProjection(){
    double w = (m_win_scale * m_win_aspect) / m_world_size;
    double h = (m_win_scale / m_win_aspect) / m_world_size;
    debug(-1) << __func__
            << "w=" << w << "h=" << h
            << "aspect=" << m_win_aspect << "scale=" << m_win_scale
            << "res=" << m_pixels_per_unit
            << "wc=" << m_win_centre;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    //glTranslatef(0.375 * w / width(), 0.375 * h / height(), 0);
    glOrtho(-w/2, w/2, -h/2, h/2, -100, 100);
    glTranslatef(m_win_centre/m_world_size);
    glMatrixMode(GL_MODELVIEW);
}

void PipelineWidget::projectionForPicking(int mouse_win_x, int mouse_win_y){
    double w = (m_win_scale * m_win_aspect) / m_world_size;
    double h = (m_win_scale / m_win_aspect) / m_world_size;
    debug(-1) << __func__
            << "w=" << w << "h=" << h
            << "aspect=" << m_win_aspect << "scale=" << m_win_scale
            << "res=" << m_pixels_per_unit
            << "wc=" << m_win_centre;
    
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    GLint viewport[4] = {0};
    glGetIntegerv(GL_VIEWPORT, viewport);
    gluPickMatrix((GLdouble)mouse_win_x, (GLdouble)(viewport[3]-mouse_win_y), 
                  1, 1, viewport);
    
    //glTranslatef(0.375 * w / width(), 0.375 * h / height(), 0);
    glOrtho(-w/2, w/2, -h/2, h/2, -100, 100);
    glTranslatef(m_win_centre/m_world_size);
    glMatrixMode(GL_MODELVIEW);
}

void PipelineWidget::drawGrid(){
    // only draw grid that will be visible:
    const double grid_major_spacing = 250;
    const double grid_minor_spacing = 25;
    
    // projected window coordinates:
    const double divisor = 2 * m_pixels_per_unit;
    const double min_x = -m_win_centre.x - width()  / divisor;
    const double min_y = -m_win_centre.y - height() / divisor;
    const double max_x = -m_win_centre.x + width()  / divisor;
    const double max_y = -m_win_centre.y + height() / divisor;
    
    const int min_grid_minor_x = roundZ(min_x / grid_minor_spacing);
    const int min_grid_minor_y = roundZ(min_y / grid_minor_spacing);
    const int max_grid_minor_x = roundZ(max_x / grid_minor_spacing);
    const int max_grid_minor_y = roundZ(max_y / grid_minor_spacing);
    
    const int min_grid_major_x = roundZ(min_x / grid_major_spacing);
    const int min_grid_major_y = roundZ(min_y / grid_major_spacing);
    const int max_grid_major_x = roundZ(max_x / grid_major_spacing);
    const int max_grid_major_y = roundZ(max_y / grid_major_spacing);
    
    debug(-2) << "min_x=" << min_x << "max_x=" << max_x
            << "min_y=" << min_y << "max_y=" << max_y << "\n\t"
            << "min_grid_minor_x=" << min_grid_minor_x
            << "max_grid_minor_x=" << max_grid_minor_x << "\n\t"
            << "min_grid_minor_y=" << min_grid_minor_y
            << "max_grid_minor_y=" << max_grid_minor_y << "\n\t"
            << "min_grid_major_x=" << min_grid_major_x
            << "max_grid_major_x=" << max_grid_major_x << "\n\t"
            << "min_grid_major_y=" << min_grid_major_y
            << "max_grid_major_y=" << max_grid_major_y;
    
    glLineWidth(1);
    glColor(Colour(0.2, 0.125));
    glBegin(GL_LINES);
    for(int i = min_grid_minor_y; i <= max_grid_minor_y; i++){
        glVertex3f(min_x, i*grid_minor_spacing, -0.2);
        glVertex3f(max_x, i*grid_minor_spacing, -0.2);
    }

    for(int i = min_grid_minor_x; i <= max_grid_minor_x; i++){
        glVertex3f(i*grid_minor_spacing, min_y, -0.2);
        glVertex3f(i*grid_minor_spacing, max_y, -0.2);
    }
    
    glColor(Colour(0.2, 0.25));
    for(int i = min_grid_major_y; i <= max_grid_major_y; i++){
        glVertex3f(min_x, i*grid_major_spacing, -0.1);
        glVertex3f(max_x, i*grid_major_spacing, -0.1);
    }

    for(int i = min_grid_major_x; i <= max_grid_major_x; i++){
        glVertex3f(i*grid_major_spacing, min_y, -0.1);
        glVertex3f(i*grid_major_spacing, max_y, -0.1);
    }
    glEnd();
}

