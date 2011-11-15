/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include "fNode.h"

#include <set>

#include <QPropertyAnimation>
#include <QGraphicsScene>

#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>
#include <liquid/arcSource.h>
#include <liquid/arc.h>

#include "elements/style.h"
#include "elements/nodeInput.h"

#include "fNodeOutput.h"
//#include "fNodeInput.h"

#include "fluidity/types.h"

using cauv::gui::f::FNode;
using namespace liquid;


// - helper structures and classes
// !!! TODO: move this to /utility or something
#include <QTextStream>
#include <QString>
struct mkQStr{
    mkQStr() : m_string(), m_stream(&m_string){ }
    template<typename T>
    mkQStr& operator<<(T const& t){
        m_stream << t;
        return *this;
    }
    operator QString() const{ return m_string; }
    QString m_string;
    QTextStream m_stream;
};

class TestLayoutItem: public QGraphicsLayoutItem,
                      public QGraphicsPathItem{
    public:
        TestLayoutItem(QRectF preferred_geom)
            : QGraphicsLayoutItem(),
              QGraphicsPathItem(),
              m_preferred_geom(preferred_geom){
            setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
            setPen(QPen(QColor(160,20,20,64)));

            setZValue(100);
            
            QPainterPath p;
            p.addRect(m_preferred_geom.adjusted(1,1,-1,-1));
            setPath(p);
        }
        virtual ~TestLayoutItem(){}
        
        void setGeometry(QRectF const& rect){
            prepareGeometryChange();
            // sets geometry()
            QGraphicsLayoutItem::setGeometry(rect);
            debug(3) << "setGeometry" << rect << "(pos=" << pos() << ")";
            setPos(rect.topLeft() - m_preferred_geom.topLeft());

            debug(3) << "parent item is" << 	parentItem();
        }

        /*void updateGeometry(){
            debug() << "updateGeometry";
        }*/

    protected:
        /*QRectF boundingRect() const{
            return geometry();
        }*/

        QSizeF sizeHint(Qt::SizeHint which, QSizeF const& constraint=QSizeF()) const{
            switch(which){
                default:
                case Qt::MinimumSize:
                case Qt::PreferredSize:
                    return m_preferred_geom.size();
                case Qt::MaximumSize:
                    return m_preferred_geom.size() * 2;
            }
        }

    private:
        QRectF m_preferred_geom;
};

class TempConnectionSink: public liquid::ConnectionSink{
    public:
        virtual bool willAcceptConnection(void* from_source){
            debug(5) << "willAcceptConnection" << from_source;
            return true;
        }

        virtual ConnectionStatus doAcceptConnection(void* from_source){
            debug(5) << "doAcceptConnection" << from_source;
            return Rejected;
        }
};

class MyConnectionSource{
    public:
        void moo(){
            debug() << "MyConnectionSource::moo";
        }
};

// - static functions
static QString nodeTypeDesc(cauv::NodeType::e const& type){
    std::string enum_name = mkStr() << type;
    return mkQStr() << enum_name.substr(enum_name.rfind(':')+1).c_str();
}

// - FNode

FNode::FNode(Manager& m, node_id_t id, NodeType::e const& type)
    : liquid::LiquidNode(F_Node_Style, NULL), 
      ManagedElement(m),
      m_node_id(id){
    initButtons();
    setType(type);    

    setSize(QSizeF(104,130));
    
    // !!!
    
    TempConnectionSink *k = new TempConnectionSink();
    MyConnectionSource *c = new MyConnectionSource();

    addItem(new liquid::ArcSink(Image_Arc_Style, Required_Image_Input, k));
    addItem(new liquid::ArcSink(Image_Arc_Style, Optional_Image_Input, k));
    //addItem(new liquid::ArcSink(Param_Arc_Style, Required_Param_Input, k));
    addItem(new liquid::ArcSink(Param_Arc_Style, Optional_Param_Input, k));

    addItem(new TestLayoutItem(QRectF(0,-5,90,10)));
    //addItem(new TestLayoutItem(QRectF(0,-5,50,10)));

    //addItem(new liquid::ArcSource(c, new liquid::Arc(Image_Arc_Style)));
    //addItem(new liquid::ArcSource(c, new liquid::Arc(Image_Arc_Style)));
    //addItem(new liquid::ArcSource(c, new liquid::Arc(Param_Arc_Style)));
    addItem(new FNodeOutput(c));
    addItem(new FNodeOutput(c));
    addItem(new FNodeOutput(c));

    m_header->setInfo(mkQStr() << id << ": 0.0MB/s 0Hz");
}


FNode::FNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p)
    : liquid::LiquidNode(F_Node_Style, NULL),
      ManagedElement(m),
      m_node_id(p->nodeId()){
    initButtons();
    setType(p->nodeType());
    m_header->setInfo(mkQStr() << p->nodeId() << ": 0.0MB/s 0Hz");

    setOutputs(p->outputs());
    setOutputLinks(p->outputs());

    setParams(p->params());
    setParamLinks(p->inputs()); // param links are inputs

    setInputs(p->inputs());
    setInputLinks(p->inputs());
}

void FNode::setType(NodeType::e const& type){
    m_header->setTitle(nodeTypeDesc(type));
}
void FNode::setInputs(msg_node_input_map_t const& inputs){
}
void FNode::setInputLinks(msg_node_input_map_t const& inputs){
}
void FNode::setOutputs(msg_node_output_map_t const& outputs){
}
void FNode::setOutputLinks(msg_node_output_map_t const& outputs){
}
void FNode::setParams(msg_node_param_map_t const& params){
}
void FNode::setParamLinks(msg_node_input_map_t const& inputs){
}

void FNode::close(){
    Q_EMIT closed(this);
    // don't delete yet! That happens when fadeAndRemove (or just remove) slots
    // are triggered
    QPropertyAnimation *fade = new QPropertyAnimation(this, "opacity");
    fade->setEndValue(0.25);
    fade->setDuration(200);    
    fade->start();
}

void FNode::fadeAndRemove(){
    debug() << "FNode::fadeAndRemove()" << this;
    disconnect();
    // now we are cut off from the outside world (apart from the scene), fade away:
    QPropertyAnimation *fade = new QPropertyAnimation(this, "opacity");
    fade->setEndValue(0);
    fade->setDuration(200);    
    fade->start();
    connect(fade, SIGNAL(finished()), this, SLOT(remove()));    
}

void FNode::remove(){
    debug() << "FNode::remove() (scene=" << scene() << ")" << this;
    scene()->removeItem(this);
    deleteLater();
}


void FNode::initButtons(){
    Button *collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), NULL, this
    );
    m_header->addButton("collapse", collapsebutton);

    Button *execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), NULL, this
    );
    m_header->addButton("exec", execbutton);
    
    Button *dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), NULL, this
    );
    m_header->addButton("duplicate", dupbutton);
}

