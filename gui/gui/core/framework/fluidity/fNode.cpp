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
#include <utility/qstring.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>
#include <liquid/arcSource.h>
#include <liquid/arc.h>

#include "elements/style.h"
#include "elements/nodeInput.h"

#include "fluidity/fNodeOutput.h"
#include "fluidity/fNodeInput.h"
#include "fluidity/manager.h"
#include "fluidity/types.h"
#include "fluidity/imageSource.h"
#include "fluidity/videoWidget.h"

using cauv::gui::f::FNode;
using cauv::gui::f::FNodeOutput;
using cauv::gui::f::FNodeInput;
using cauv::mkQStr;
using namespace liquid;


// - helper structures and classes

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
            Q_UNUSED(constraint);
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


// - static functions
static QString nodeTypeDesc(cauv::NodeType::e const& type){
    std::string enum_name = mkStr() << type;
    return mkQStr() << enum_name.substr(enum_name.rfind(':')+1).c_str();
}

static int countLocalInputsWithName(std::string const& name, FNode::msg_node_param_map_t const& map){
    foreach(FNode::msg_node_param_map_t::value_type const& v, map)
        if(v.first.input == name)
            return 1;
    return 0;
}

// - FNode

FNode::FNode(Manager& m, node_id_t id, NodeType::e const& type)
    : liquid::LiquidNode(F_Node_Style, NULL), 
      ManagedElement(m),
      m_node_id(id),
      m_type(type){
    initButtons();
    setType(type);

    setSize(QSizeF(104,130));
    
    // !!!

    m_header->setInfo(mkQStr() << id << ": 0.0MB/s 0Hz");
}


FNode::FNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p)
    : liquid::LiquidNode(F_Node_Style, NULL),
      ManagedElement(m),
      m_node_id(p->nodeId()),
      m_type(p->nodeType()){
    initButtons();
    setType(p->nodeType());
    m_header->setInfo(mkQStr() << p->nodeId() << ": 0.0MB/s 0Hz");

    setParams(p->params());
    setParamLinks(p->inputs()); // param links are inputs

    setInputs(p->inputs());
    setInputLinks(p->inputs());
    
    setOutputs(p->outputs());
    setOutputLinks(p->outputs());
}

void FNode::setType(NodeType::e const& type){
    m_type = type;
    m_header->setTitle(nodeTypeDesc(type));
}

void FNode::setInputs(msg_node_input_map_t const& inputs){
    for(str_in_map_t::const_iterator i = m_inputs.begin(); i != m_inputs.end(); i++)
        removeItem(i->second);
    m_inputs.clear();
    msg_node_input_map_t::const_iterator j;
    for(j = inputs.begin(); j != inputs.end(); j++){
        // Don't add any inputs that are actually parameters! Note that
        // setParams must be called first, so this information is available.
        if(!m_params.count(j->first.input)){
            debug() << BashColour::Blue << "FNode:: new input:" << *j;
            FNodeInput* t = new FNodeImageInput(manager(), j->first, this);
            m_inputs[j->first.input] = t;
            addItem(t);
        }
    }
}

void FNode::setInputLinks(msg_node_input_map_t const& inputs){
    m_input_links.clear();
    msg_node_input_map_t::const_iterator j;
    for(j = inputs.begin(); j != inputs.end(); j++){
        str_in_map_t::const_iterator k = m_inputs.find(j->first.input);
        // if connected, add the arc to this input: 
        if(!j->second.node) // not connected
            continue;
        if(k == m_inputs.end()) // parameter, not an input
            continue;
        m_input_links.push_back(NodeInputArc(j->first.input, j->second));

        fnode_ptr from = manager().lookup(j->second.node);
        FNodeOutput* output = NULL;
        // !!! TODO:
        //k->second->disconnect();
        if(from && (output = from->output(j->second.output)))
            output->arc()->addTo(k->second->sink());
    }
}

void FNode::setOutputs(msg_node_output_map_t const& outputs){
    for(str_out_map_t::const_iterator i = m_outputs.begin(); i != m_outputs.end(); i++)
        removeItem(i->second);
    m_outputs.clear();
    
    msg_node_output_map_t::const_iterator i;
    for(i = outputs.begin(); i != outputs.end(); i++){
        debug() << BashColour::Blue << "FNode:: new output:" << *i;
        FNodeOutput* t;
        if(i->first.type == OutputType::Image)
            t = new FNodeImageOutput(i->first, this);
        else
            t = new FNodeParamOutput(i->first, this);
        m_outputs[i->first.output] = t;
        addItem(t);
    }
}

void FNode::setOutputLinks(msg_node_output_map_t const& outputs){
    // don't construct any output links - that would be completely redundant,
    // but do record what out output links are, so this node can be duplicated
    m_output_links.clear();
    msg_node_output_map_t::const_iterator j;
    for(j = outputs.begin(); j != outputs.end(); j++){
        msg_node_in_list_t::const_iterator i;
        for(i = j->second.begin(); i != j->second.end(); i++){
            m_output_links.push_back(NodeOutputArc(*i, j->first.output));
        }
    }
    
}

void FNode::setParams(msg_node_param_map_t const& params){
    str_inparam_map_t new_m_params;
    foreach(str_inparam_map_t::value_type const &i, m_params)
        if(!countLocalInputsWithName(i.first, params)){
            removeItem(i.second);
        }else{
            new_m_params.insert(i);
        }
    m_params = new_m_params;
    foreach(msg_node_param_map_t::value_type const& j, params){
        str_inparam_map_t::iterator k = m_params.find(j.first.input);    
        if(k == m_params.end()){
            debug() << BashColour::Blue << "FNode:: new param:" << j;        
            FNodeParamInput* t = new FNodeParamInput(manager(), j.first, this);
            m_params[j.first.input] = t;
            addItem(t);
            t->setValue(j.second);
        }else{
            debug() << BashColour::Blue << "FNode:: update param:" << j;
            FNodeParamInput* t = dynamic_cast<FNodeParamInput*>(k->second);
            if(t){
                t->setValue(j.second);
            }else{
                error() << j.first << "is not a parameter! Value cannot be set.";
            }
        }
    }

}

void FNode::setParamLinks(msg_node_input_map_t const& inputs){
    typedef msg_node_input_map_t im_t;
    foreach(im_t::value_type const& j, inputs){
        str_inparam_map_t::const_iterator k = m_params.find(j.first.input);
        // if connected, add the arc to this parameter's input:
        if(k == m_params.end()) // input not param
            continue;
        if(j.second.node){
            m_input_links.push_back(NodeInputArc(j.first.input, j.second));
            fnode_ptr from = manager().lookup(j.second.node);
            FNodeOutput* output = NULL;
            if(from && (output = from->output(j.second.output)))
                output->arc()->addTo(k->second->sink());
            k->second->setEditable(false);
        }else{
            k->second->setEditable(true);            
        }
    }
}

void FNode::connectOutputTo(std::string const& output_id, fnode_ptr to, std::string const& input_id){
    FNodeOutput* output = this->output(output_id);
    FNodeInput* input = to->input(input_id);
    if(output && input)
        // !!! TODO: input->disconnect()
        output->arc()->addTo(input->sink());
}

void FNode::disconnectOutputFrom(std::string const& output_id, fnode_ptr to, std::string const& input_id){
    FNodeOutput* output = this->output(output_id);
    FNodeInput* input = to->input(input_id);
    if(output && input)
        output->arc()->removeTo(input->sink());
}

void FNode::addImageDisplayOnInput(std::string const& input, boost::shared_ptr<ImageSource> src){
    str_in_map_t::const_iterator i = m_inputs.find(input);
    if(i == m_inputs.end()){
        error() << "no such input (yet)" << input;
        return;
    }
    
    VideoWidget* w = new VideoWidget(i->second);
    connect(src.get(), SIGNAL(newImageAvailable(boost::shared_ptr<const GuiImageMessage>)),
            w, SLOT(displayImage(boost::shared_ptr<const GuiImageMessage>)));
    i->second->addWidget(w);
    setResizable(true);
}

void FNode::close(){
    Q_EMIT LiquidNode::closed(this);
    Q_EMIT closed(m_node_id);
    // don't delete yet! That happens when fadeAndRemove (or just remove) slots
    // are triggered
    if(manager().animationPermitted()){
        QPropertyAnimation *fade = new QPropertyAnimation(this, "opacity");
        fade->setEndValue(0.25);
        fade->setDuration(200);
        fade->start();
    }else{
        setOpacity(0.25);
    }
}

void FNode::fadeAndRemove(){
    debug() << "FNode::fadeAndRemove()" << this;
    disconnect();
    // now we are cut off from the outside world (apart from the scene), fade away:
    if(manager().animationPermitted()){
        QPropertyAnimation *fade = new QPropertyAnimation(this, "opacity");
        fade->setEndValue(0);
        fade->setDuration(200);    
        fade->start();
        connect(fade, SIGNAL(finished()), this, SLOT(remove()));
    }else{
        remove();
    }
}

void FNode::remove(){
    debug() << "FNode::remove() (scene=" << scene() << ")" << this;
    scene()->removeItem(this);
    deleteLater();
}

void FNode::reExec(){
    manager().requestForceExec(id());
}

void FNode::duplicate(){
    // Don't duplicate output links - this would break the connections of the
    // duplicated node
    manager().requestNode(m_type, m_input_links/*, m_output_links*/);
}

FNodeOutput* FNode::output(std::string const& id){
    str_out_map_t::const_iterator i = m_outputs.find(id);
    if(i != m_outputs.end())
        return i->second;
    error() << "no such output: " << id;
    return NULL;
}

FNodeInput* FNode::input(std::string const& id){
    str_in_map_t::const_iterator i = m_inputs.find(id);
    if(i != m_inputs.end())
        return i->second;
    str_inparam_map_t::const_iterator j = m_params.find(id);
    if(j != m_params.end())
        return j->second;
    error() << "no such input: " << id;
    return NULL;
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
    
    connect(this, SIGNAL(closed(node_id_t const&)), &manager(), SLOT(requestRemoveNode(node_id_t const&)));
    connect(dupbutton, SIGNAL(pressed()), this, SLOT(duplicate()));
    connect(execbutton, SIGNAL(pressed()), this, SLOT(reExec()));
}

