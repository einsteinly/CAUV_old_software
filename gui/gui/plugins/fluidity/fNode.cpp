/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "fNode.h"

#include <set>
#include <ios>
#include <iomanip>

#include <QPropertyAnimation>
#include <QGraphicsScene>

#include <debug/cauv_debug.h>
#include <utility/qt_streamops.h>
#include <utility/qstring.h>
#include <utility/time.h>
#include <utility/streamops/pair.h>
#include <utility/string.h>

#include <liquid/button.h>
#include <liquid/nodeHeader.h>
#include <liquid/connectionSink.h>
#include <liquid/arcSink.h>
#include <liquid/arcSource.h>
#include <liquid/arc.h>

#include <widgets/videoWidget.h>

#include "elements/style.h"
#include "elements/nodeInput.h"

#include "fNodeOutput.h"
#include "fNodeInput.h"
#include "manager.h"
#include "types.h"
//#include "imageSource.h"

#include "model/nodes/groupingnode.h"
#include "model/paramvalues.h"

using cauv::gui::f::FNode;
using cauv::gui::f::FNodeOutput;
using cauv::gui::f::FNodeInput;
using cauv::mkQStr;
using namespace liquid;
using namespace cauv;
using namespace cauv::gui;

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
            CAUV_LOG_DEBUG(3, "setGeometry" << rect << "(pos=" << pos() << ")");
            setPos(rect.topLeft() - m_preferred_geom.topLeft());

            CAUV_LOG_DEBUG(3, "parent item is" << parentItem());
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
#warning old node type stuff
#if 0
static QString nodeTypeDesc(cauv::NodeType::e const& type){
    std::string enum_name = mkStr() << type;
    return mkQStr() << enum_name.substr(enum_name.rfind(':')+1).c_str();
}
#endif

// static int countLocalInputsWithName(const std::string& name, FNode::msg_node_param_map_t const& map){
//     for (FNode::msg_node_param_map_t::value_type const& v : map)
//         if(v.first.input == name)
//             return 1;
//     return 0;
// }


// - FNode

FNode::FNode(boost::shared_ptr<GuiNodeModel> node, Manager &m)
    :liquid::LiquidNode(F_Node_Style(), nullptr), 
      ManagedElement(m),
      m_associated_node(node),
      m_collapsed(false),
      m_name(m_associated_node->getName()){

    setTitle(QString::fromStdString(m_associated_node->getName()));
    initButtons();
    initIO();

    setSize(QSizeF(104,130));
    
    status(OK);

    //create grouping node in model tree
    manager().model()->findOrCreate<GroupingNode>(getName());
    
#ifdef QT_PROFILE_GRAPHICSSCENE
    setProfileName("FNode");
#endif // def QT_PROFILE_GRAPHICSSCENE
    CAUV_LOG_DEBUG(2, "Created FNode for pipeline model node " << m_associated_node->getName());
}

FNode::~FNode(){
    CAUV_LOG_DEBUG(2, "Destroyed FNode for pipeline model node " << m_associated_node->getName());
    
    //destroy grouping node in model tree
    manager().model()->removeChild(getName());
}

void FNode::initIO() {
    for (auto &output_name : m_associated_node->getOutputNames()) {
        //makes sure output is created
        auto output = m_associated_node->getOutput(output_name);
        
        auto t = new FNodeOutput(output_name, *this, Param_Arc_Style());
        m_outputs.insert(std::make_pair(output_name, t));
        addItem(t);
    }

    for (auto &input_name : m_associated_node->getInputNames()) {
        //make sure input is created
        auto input = m_associated_node->getInput(input_name);
        
        auto t = new FNodeInput(input_name, *this, manager());
        t->setValue(input.value);
        //editable (until connection occurs)
        t->setEditable(true);
        
        connect(t, SIGNAL(modelValueChanged(const std::string&, QVariant)),
                this, SLOT(modelParamValueChanged(const std::string&, QVariant)));
        
        m_inputs.insert(std::make_pair(input_name, t));
        addItem(t);
    }
}
        
void FNode::constructArcTo(const std::string output, FNode* to, const std::string input){
    FNodeInput* in = to->getInput(input);
    in->setEditable(false);
    getOutput(output)->arc()->addTo(in->sink());
}

void FNode::destructArcFrom(const std::string input, FNode* from, const std::string output){
    //set input as editable
    getInput(input)->setEditable(true);
    //destruction might happen when we have no FNode so be careful
    if (!from) { return; }
    from->getOutput(output)->arc()->removeTo(getInput(input)->sink());
}

FNodeInput* FNode::getInput(const std::string input_name){
    return m_inputs.at(input_name);
}

FNodeOutput* FNode::getOutput(const std::string output_name){
    return m_outputs.at(output_name);
}
        
// FNode::FNode(Manager& m, boost::shared_ptr<NodeAddedMessage const> p)
//     : liquid::LiquidNode(F_Node_Style(), nullptr),
//       ManagedElement(m),
//       m_node_id(p->nodeId()),
//       m_type(p->nodeType()),
//       m_collapsed(false){
//     initButtons();
//     setType(p->nodeType());
// 
//     setParams(p->params());
//     setParamLinks(p->inputs()); // param links are inputs
// 
//     setInputs(p->inputs());
//     setInputLinks(p->inputs());
//     
//     setOutputs(p->outputs());
//     setOutputLinks(p->outputs());
// 
//     status(OK);
// 
// #ifdef QT_PROFILE_GRAPHICSSCENE
//     setProfileName("FNode");
// #endif // def QT_PROFILE_GRAPHICSSCENE
// 
// }

// void FNode::setType(NodeType::e const& type){
//     m_type = type;
//     setTitle(nodeTypeDesc(type));
// }
// 
// void FNode::setInputs(msg_node_input_map_t const& inputs){
//     for(str_in_map_t::const_iterator i = m_inputs.begin(); i != m_inputs.end(); i++)
//         removeItem(i->second);
//     m_inputs.clear();
//     msg_node_input_map_t::const_iterator j;
//     for(j = inputs.begin(); j != inputs.end(); j++){
//         // Don't add any inputs that are actually parameters! Note that
//         // setParams must be called first, so this information is available.
//         if(!m_params.count(j->first.input)){
//             debug() << BashColour::Blue << "FNode:: new input:"
//                     << std::string(mkStr().lengthLimit(180) << *j);
//             FNodeInput* t = new FNodeImageInput(manager(), j->first, this);
//             m_inputs[j->first.input] = t;
//             addItem(t);
//         }
//     }
// }
// 
// void FNode::setInputLinks(msg_node_input_map_t const& inputs){
//     m_input_links.clear();
//     msg_node_input_map_t::const_iterator j;
//     for(j = inputs.begin(); j != inputs.end(); j++){
//         str_in_map_t::const_iterator k = m_inputs.find(j->first.input);
//         // if connected, add the arc to this input: 
//         if(!j->second.node) // not connected
//             continue;
//         if(k == m_inputs.end()) // parameter, not an input
//             continue;
//         m_input_links.push_back(NodeInputArc(j->first.input, j->second));
// 
//         fnode_ptr from = manager().lookup(j->second.node);
//         FNodeOutput* output = nullptr;
//         // !!! TODO:
//         //k->second->disconnect();
//         if(from && (output = from->output(j->second.output)))
//             output->arc()->addTo(k->second->sink());
//     }
// }
// 
// void FNode::setOutputs(msg_node_output_map_t const& outputs){
//     for(str_out_map_t::const_iterator i = m_outputs.begin(); i != m_outputs.end(); i++)
//         removeItem(i->second);
//     m_outputs.clear();
//     
//     msg_node_output_map_t::const_iterator i;
//     for(i = outputs.begin(); i != outputs.end(); i++){
//         debug() << BashColour::Blue << "FNode:: new output:"
//                 << std::string(mkStr().lengthLimit(180) << *i);
//         FNodeOutput* t;
//         if(i->first.type == OutputType::Image)
//             t = new FNodeImageOutput(i->first, this);
//         else
//             t = new FNodeParamOutput(i->first, this);
//         m_outputs[i->first.output] = t;
//         addItem(t);
//     }
// }
// 
// void FNode::setOutputLinks(msg_node_output_map_t const& outputs){
//     // don't construct any output links - that would be completely redundant,
//     // but do record what out output links are, so this node can be duplicated
//     m_output_links.clear();
//     msg_node_output_map_t::const_iterator j;
//     for(j = outputs.begin(); j != outputs.end(); j++){
//         msg_node_in_list_t::const_iterator i;
//         for(i = j->second.begin(); i != j->second.end(); i++){
//             m_output_links.push_back(NodeOutputArc(*i, j->first.output));
//         }
//     }
//     
// }
// 
// void FNode::setParams(msg_node_param_map_t const& params){
//     str_inparam_map_t new_m_params;
//     for (str_inparam_map_t::value_type const &i : m_params)
//         if(!countLocalInputsWithName(i.first, params)){
//             removeItem(i.second);
//         }else{
//             new_m_params.insert(i);
//         }
//     m_params = new_m_params;
//     for (msg_node_param_map_t::value_type const& j : params){
//         auto k = m_params.find(j.first.input);    
//         if(k == m_params.end()){
//             debug() << BashColour::Blue << "FNode:: new param:"
//                     << std::string(mkStr().lengthLimit(180) << j);
//             auto  t = new FNodeParamInput(manager(), j.first, this);
//             m_params[j.first.input] = t;
//             addItem(t);
//             t->setValue(j.second);
//         }else{
//             debug() << BashColour::Blue << "FNode:: update param:"
//                     << std::string(mkStr().lengthLimit(180) << j);
//             FNodeParamInput* t = dynamic_cast<FNodeParamInput*>(k->second);
//             if(t){
//                 t->setValue(j.second);
//             }else{
//                 error() << j.first << "is not a parameter! Value cannot be set.";
//             }
//         }
//     }
// 
// }
// 
// void FNode::setParamLinks(msg_node_input_map_t const& inputs){
//     typedef msg_node_input_map_t im_t;
//     for (im_t::value_type const& j : inputs){
//         str_inparam_map_t::const_iterator k = m_params.find(j.first.input);
//         // if connected, add the arc to this parameter's input:
//         if(k == m_params.end()) // input not param
//             continue;
//         if(j.second.node){
//             m_input_links.push_back(NodeInputArc(j.first.input, j.second));
//             fnode_ptr from = manager().lookup(j.second.node);
//             FNodeOutput* output = nullptr;
//             if(from && (output = from->output(j.second.output)))
//                 output->arc()->addTo(k->second->sink());
//             k->second->setEditable(false);
//         }else{
//             k->second->setEditable(true);            
//         }
//     }
// }
// 
// void FNode::connectOutputTo(const std::string& output_id, fnode_ptr to, const std::string& input_id){
//     FNodeOutput* output = this->output(output_id);
//     FNodeInput* input = to->input(input_id);
//     if(output && input)
//         // !!! TODO: input->disconnect()
//         output->arc()->addTo(input->sink());
// }
// 
// void FNode::disconnectOutputFrom(const std::string& output_id, fnode_ptr to, const std::string& input_id){
//     FNodeOutput* output = this->output(output_id);
//     FNodeInput* input = to->input(input_id);
//     if(output && input)
//         output->arc()->removeTo(input->sink());
// }
// 
// void FNode::addImageDisplayOnInput(const std::string& input, boost::shared_ptr<ImageSource> src){
//     str_in_map_t::const_iterator i = m_inputs.find(input);
//     if(i == m_inputs.end()){
//         error() << "no such input (yet)" << input;
//         return;
//     }
//     
//     auto  w = new VideoWidget(i->second);
//     connect(src.get(), SIGNAL(newImageAvailable(boost::shared_ptr<const GuiImageMessage>)),
//             w, SLOT(displayImage(boost::shared_ptr<const GuiImageMessage>)));
//     i->second->addWidget(w);
//     setResizable(true);
// }

void FNode::status(Status const& s, const std::string& status_information){
    if(!status_information.size())
        LiquidNode::status(s, mkStr() << m_associated_node->id << ": unknown status");
    else
        LiquidNode::status(s, mkStr() << m_associated_node->id << ": " << status_information);
}

void FNode::status(Status const& s, float const& throughput, float const& frequency, float const& time_taken, float const& time_ratio){
    LiquidNode::status(
        s,
        mkStr()
            << m_associated_node->id << ": "
            << std::setprecision(1) << std::setiosflags(std::ios::fixed) << throughput << "Mb/s "
            << std::setprecision(1) << std::setiosflags(std::ios::fixed) << frequency << "Hz "
            << std::setprecision(0) << std::setiosflags(std::ios::fixed) << time_taken << "ms "
            << std::setprecision(0) << std::setiosflags(std::ios::fixed) << time_ratio*100 << "%"
    );
}

//called when we hit the close button
void FNode::close(){
    //tells manager etc to clear up these nodes
    Q_EMIT LiquidNode::closed(this);
    Q_EMIT closed(*this);
}

//actually remove this node (take ownership)
FNode* FNode::remove(){
    CAUV_LOG_DEBUG(1, "FNode::remove()" << this);
    disconnect();
    scene()->removeItem(this);
    return this;
}

void FNode::reExec(){
//     manager().requestForceExec(id());
}

void FNode::duplicate(){
//     // Don't duplicate output links - this would break the connections of the
//     // duplicated node
//     debug() << "duplicate()" << m_type << m_input_links;
//     manager().requestNode(m_type, m_input_links/*, m_output_links*/);
}

void FNode::toggleCollapsed(){
    m_collapsed = !m_collapsed;
#if 0
    for(int i = 0; i < m_contentLayout->count(); i++){
        FNodeInput* ip = dynamic_cast<FNodeInput*>(m_contentLayout->itemAt(i)->graphicsItem());
        if(ip) {
            ip->setCollapsed(m_collapsed);
        }
    }
#endif
    setSize(Minimum_Size);
    manager().considerUpdatingLayout();
}

void FNode::modelParamValueChanged(const std::string& input, QVariant variant){
    m_associated_node->setInputValue(input, qVariantToParamValue(variant));
}
 
// FNodeOutput* FNode::output(const std::string& id){
//     str_out_map_t::const_iterator i = m_outputs.find(id);
//     if(i != m_outputs.end())
//         return i->second;
//     error() << "no such output: " << id;
//     return nullptr;
// }
// 
// FNodeInput* FNode::input(const std::string& id){
//     str_in_map_t::const_iterator i = m_inputs.find(id);
//     if(i != m_inputs.end())
//         return i->second;
//     str_inparam_map_t::const_iterator j = m_params.find(id);
//     if(j != m_params.end())
//         return j->second;
//     error() << "no such input: " << id;
//     return nullptr;
// }

void FNode::initButtons(){
    Button *collapsebutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/collapse_button"), nullptr, this
    );
    addButton("collapse", collapsebutton);

    Button *execbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/reexec_button"), nullptr, this
    );
    addButton("exec", execbutton);
    
    Button *dupbutton = new Button(
       QRectF(0,0,24,24), QString(":/resources/icons/dup_button"), nullptr, this
    );
    addButton("duplicate", dupbutton);
    
    connect(this, SIGNAL(closed(FNode&)), &manager(), SLOT(removeNode(FNode&)));
    connect(dupbutton, SIGNAL(pressed()), this, SLOT(duplicate()));
    connect(execbutton, SIGNAL(pressed()), this, SLOT(reExec()));
    connect(collapsebutton, SIGNAL(pressed()), this, SLOT(toggleCollapsed()));
}

