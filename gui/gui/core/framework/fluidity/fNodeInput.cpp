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

#include "fNodeInput.h" 

#include <QLabel>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>

#include <liquid/arcSink.h>
#include <liquid/proxyWidget.h>

#include <gui/core/model/node.h>
#include <gui/core/model/variants.h>
#include <gui/core/model/nodes/numericnode.h>
#include <gui/core/model/nodes/stringnode.h>
#include <gui/core/model/singleItemModel.h>
#include <gui/core/framework/nodepicker.h>

#include <debug/cauv_debug.h>

#include "fluidity/manager.h"
#include "fluidity/fNodeOutput.h"

#include <generated/types/LocalNodeInput.h>
#include <generated/types/SetNodeParameterMessage.h>
#include <generated/types/ParamValueType.h>

using namespace cauv;
using namespace cauv::gui::f;

// - FNodeInput
FNodeInput::FNodeInput(Manager& m,
                       liquid::ArcStyle const& of_style,
                       liquid::CutoutStyle const& with_cutout,
                       FNode* node,
                       std::string const& id)
    : ArcSinkLabel(new liquid::ArcSink(of_style, with_cutout, this),
                   node,
                   QString::fromStdString(id)),
      FNodeIO(node, id),
      ManagedElement(m){
}

FNodeInput::~FNodeInput(){
}

void FNodeInput::setCollapsed(bool state){
    Q_UNUSED(state);
}

void FNodeInput::addWidget(QGraphicsWidget* w){
    vLayout()->addItem(w);
}

void FNodeInput::removeWidget(QGraphicsWidget* w){
    vLayout()->removeItem(w);
}

bool FNodeInput::willAcceptConnection(liquid::ArcSourceDelegate* from_source){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
    debug(7) << "fNodeInput::willAcceptConnection from_source=" << from_source << output
             << (output && (output->ioType() == ioType() && output->subType() == subType() && output->node() != node()));
    if(output)
        return output->ioType() == ioType() &&
               output->subType() == subType() &&
               output->node() != node();
    return false;
}
FNodeInput::ConnectionStatus FNodeInput::doAcceptConnection(liquid::ArcSourceDelegate* from_source){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
    debug() << "FNodeInput::doAcceptConnection:" << output;
    if(output &&
       output->ioType() == ioType() &&
       // for now, allow people to try to force connections to the wrong place if they really want to
       //output->subType() == subType() && 
       output->node() != node()){
        manager().requestArc(
            NodeOutput(output->node()->id(), output->id(), ioType(), subType()),
            NodeInput(node()->id(), id(), subType())
        );
        return Pending;
    }
    return Rejected;
}


// - FNodeImageInput
FNodeImageInput::FNodeImageInput(Manager& m, LocalNodeInput const& input, FNode* node)
    : FNodeInput(m, Image_Arc_Style, cutoutStyleForSchedType(input.schedType), node, input.input){
}

OutputType::e FNodeImageInput::ioType() const{
    return OutputType::Image;
}

SubType FNodeImageInput::subType() const{
    return -1;
}

liquid::CutoutStyle const& FNodeImageInput::cutoutStyleForSchedType(InputSchedType::e const& st){
    switch(st){
        case InputSchedType::Must_Be_New: return Required_Image_Input;
        case InputSchedType::May_Be_Old:  return Optional_Image_Input;
        case InputSchedType::Optional:    return Optional_Image_Input;
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Image_Input;
    }
}

// - static helper stuff:
struct MakeModelNode: boost::static_visitor<boost::shared_ptr<cauv::gui::Node> >{
    MakeModelNode(std::string const& id) : id(id) { }
    
    // catch-all for uneditable types...
    template<typename T>
    boost::shared_ptr<cauv::gui::Node> operator()(T const&) const{
        debug(7) << "MakeModelNode: catch all (string)";    
        return boost::make_shared< cauv::gui::StringNode >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(float const&) const{
        debug(7) << "MakeModelNode: float";
        return boost::make_shared< cauv::gui::NumericNode<float> >(cauv::gui::nid_t(id));
    }
    
    boost::shared_ptr<cauv::gui::Node> operator()(bool const&) const{
        debug(7) << "MakeModelNode: bool";    
        return boost::make_shared< cauv::gui::NumericNode<bool> >(cauv::gui::nid_t(id));
    }
    
    boost::shared_ptr<cauv::gui::Node> operator()(int32_t const&) const{
        debug(7) << "MakeModelNode: int";    
        return boost::make_shared< cauv::gui::NumericNode<int32_t> >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(cauv::BoundedFloat const&) const{
        debug(7) << "MakeModelNode: BoundedFloat";    
        return boost::make_shared< cauv::gui::NumericNode<cauv::BoundedFloat> >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(std::string const&) const{
        debug(7) << "MakeModelNode: string";    
        // !!! need a non-editable node
        return boost::make_shared<cauv::gui::StringNode>(cauv::gui::nid_t(id));
    }

    private:
        std::string const& id;
};

static boost::shared_ptr<cauv::gui::Node> makeModelNodeForInput(std::string const& id, ParamValue const& v){
    return boost::apply_visitor(MakeModelNode(id), v);
}


// - FNodeParamInput
FNodeParamInput::FNodeParamInput(Manager& m, LocalNodeInput const& input, FNode* node)
    : FNodeInput(m, Image_Arc_Style, cutoutStyleForSchedType(input.schedType), node, input.input),
      m_subtype(input.subType),
      m_compatible_subtypes(input.compatibleSubTypes.begin(), input.compatibleSubTypes.end()),
      m_model(),
      m_model_node(),
      m_view(NULL),
      m_view_proxy(NULL),
      m_value(){
}
FNodeParamInput::~FNodeParamInput(){
    // if it isn't in the layout, then need to delete the view proxy
    for(int i = 0; i < vLayout()->count(); i++)
        if(vLayout()->itemAt(i) == m_view_proxy)
            return;
    m_view_proxy->deleteLater();
}

OutputType::e FNodeParamInput::ioType() const{
    return OutputType::Parameter;
}

SubType FNodeParamInput::subType() const{ return m_subtype;
}

void FNodeParamInput::setValue(ParamValue const& v){
    m_value = v;
    if(!m_model){
        m_model_node = makeModelNodeForInput(id(), v);
        m_model = new SingleNodeItemModel(m_model_node);
        connect(m_model_node.get(), SIGNAL(onSet(QVariant)), this, SLOT(modelValueChanged(QVariant)));
        initView();
    }
    m_model_node->update(variantToQVariant(v));
}

void FNodeParamInput::setCollapsed(bool collapsed){
    if(collapsed){
        removeWidget(m_view_proxy);
        m_view_proxy->hide();
    }else{
        addWidget(m_view_proxy);
        m_view_proxy->show();
    }
    // update the layout here before returning to caller (so everything is
    // up-to-date)
    vLayout()->activate();
}

void FNodeParamInput::setEditable(bool editable){
    m_model_node->setMutable(editable);
}

bool FNodeParamInput::willAcceptConnection(liquid::ArcSourceDelegate* from_source){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
    debug(7) << "FNodeParamInput::willAcceptConnection from_source=" << from_source << output
             << (output && (output->ioType() == ioType() && m_compatible_subtypes.count(output->subType()) && output->node() != node()));
    if(output)
        return output->ioType() == ioType() &&
               m_compatible_subtypes.count(output->subType()) &&
               output->node() != node();
    return false;
}

void FNodeParamInput::modelValueChanged(QVariant value){
    debug() << "modelValueChanged:" << id() << value.typeName();
    ParamValue pv;
    try{
        // !!! TODO: this is a workaround for the fact that the true type isn't
        // available in the onUpdate signal
        pv = m_value;
        if(pv.which() == ParamValueType::BoundedFloatType){
            boost::get<BoundedFloat>(pv).value = value.toFloat();
        }else{
            pv = qVariantToVariant<ParamValue>(value);
        }
    }catch(std::bad_cast& e){
        error() << "modelValueChanged:" << e.what()
                << "value type:" << value.typeName()
                << "expected types:" << m_compatible_subtypes;
        return;
    }
    manager().sendMessage(boost::make_shared<SetNodeParameterMessage>(
        manager().pipelineName(),
        node()->id(),
        id(),
        pv
    ));
}

liquid::CutoutStyle const& FNodeParamInput::cutoutStyleForSchedType(InputSchedType::e const& st){
    switch(st){
        case InputSchedType::Must_Be_New: return Required_Param_Input;
        case InputSchedType::May_Be_Old:  return Optional_Param_Input;
        case InputSchedType::Optional:    return Optional_Param_Input;
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Param_Input;
    }
}

void FNodeParamInput::initView(){
    assert(m_model);

    m_view = new NodeTreeView();
    m_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    m_view->setMinimumSize(QSize(60,21));
    m_view->setMaximumSize(QSize(1000,21));
    m_view->setModel(m_model);
    m_view->setColumnWidth(0,80);
    //m_view->resizeRowsToContents();

    // umm, doesn't play well with editing widgets!
    //m_view->setStyleSheet("QTreeView {background-color: transparent;}");

    // And this seems to cause weird corruption (but looks just right) :(
    //m_view->setStyleSheet("QTreeView {background-color: #f3f3f3; border: 0px; padding: 0px;}");

    // playing around...
    //m_view->setStyleSheet("QTreeView::item { border: 0px; padding: 0px; }");
    //m_view->setStyleSheet("QTreeView::item {background: transparent;}");
    //m_view->setStyleSheet("QTreeView::item {background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #e7effd, stop: 1 #cbdaf1);}");

    m_view_proxy = new liquid::ProxyWidget();
    m_view_proxy->setWidget(m_view);
    m_view_proxy->setWidget(m_view);
    
    addWidget(m_view_proxy);
}

