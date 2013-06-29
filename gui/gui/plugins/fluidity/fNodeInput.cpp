/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "fNodeInput.h" 

#include <QLabel>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>

#include <utility/streamops/set.h>
#include <debug/cauv_debug.h>

#include <liquid/arcSink.h>
#include <liquid/proxyWidget.h>

#include <model/node.h>
#include <model/variants.h>
#include <model/nodes/numericnode.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/stringnode.h>
#include <model/nodes/colournode.h>
#include <model/nodeItemModel.h>
#include <nodepicker.h>

#include "manager.h"
#include "fNodeOutput.h"

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
    w->setParent(nullptr);
    hLayout()->addItem(w);
}

void FNodeInput::removeWidget(QGraphicsWidget* w){
    hLayout()->removeItem(w);
}

bool FNodeInput::willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
    debug(7) << "fNodeInput::willAcceptConnection from_source=" << from_source << output
             << (output && (output->ioType() == ioType() && output->subType() == subType() && output->node() != node()));
    if(output)
        return output->ioType() == ioType() &&
               output->subType() == subType() &&
               output->node() != node();
    return false;
}
FNodeInput::ConnectionStatus FNodeInput::doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*){
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
    : FNodeInput(m, Image_Arc_Style(), cutoutStyleForSchedType(input.schedType), node, input.input){
}

OutputType::e FNodeImageInput::ioType() const{
    return OutputType::Image;
}

SubType FNodeImageInput::subType() const{
    return -1;
}

liquid::CutoutStyle const& FNodeImageInput::cutoutStyleForSchedType(InputSchedType::e const& st){
    switch(st){
        case InputSchedType::Must_Be_New: return Required_Image_Input();
        case InputSchedType::May_Be_Old:  return Optional_Image_Input();
        case InputSchedType::Optional:    return Optional_Image_Input();
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Image_Input();
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
        typedef cauv::gui::NumericNode<float> float_node_t;
        boost::shared_ptr<float_node_t> r = boost::make_shared<float_node_t>(cauv::gui::nid_t(id));
        r->setPrecision(8);
        return r;
    }
    
    boost::shared_ptr<cauv::gui::Node> operator()(bool const&) const{
        debug(7) << "MakeModelNode: bool";    
        return boost::make_shared< cauv::gui::BooleanNode >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(Colour const&) const{
        debug(7) << "MakeModelNode: Colour";
        return boost::make_shared< cauv::gui::ColourNode >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(int32_t const&) const{
        debug(7) << "MakeModelNode: int";
        return boost::make_shared< cauv::gui::NumericNode<int32_t> >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(cauv::BoundedFloat const&) const{
        debug(7) << "MakeModelNode: BoundedFloat";
        typedef cauv::gui::NumericNode<cauv::BoundedFloat> boundedfloat_node_t;
        boost::shared_ptr<boundedfloat_node_t> r = boost::make_shared<boundedfloat_node_t>(cauv::gui::nid_t(id));
        r->setPrecision(5);
        return r;
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
    : FNodeInput(m, Image_Arc_Style(), cutoutStyleForSchedType(input.schedType), node, input.input),
      m_subtype(input.subType),
      m_compatible_subtypes(input.compatibleSubTypes.begin(), input.compatibleSubTypes.end()),
      m_model_node(),
      m_view(nullptr),
      m_view_proxy(nullptr){
    // don't want a label:
    removeWidget(m_text);
    m_text->deleteLater();
    m_text = nullptr;
}
FNodeParamInput::~FNodeParamInput(){
    // if it isn't in the layout, then need to delete the view proxy
    for(int i = 0; i < hLayout()->count(); i++)
        if(hLayout()->itemAt(i) == m_view_proxy)
            return;
    m_view_proxy->deleteLater();
}

OutputType::e FNodeParamInput::ioType() const{
    return OutputType::Parameter;
}

SubType FNodeParamInput::subType() const{
    return m_subtype;
}

void FNodeParamInput::setValue(ParamValue const& v){
    if(!m_model_node){
        m_model_node = makeModelNodeForInput(id(), v);
        boost::shared_ptr<gui::Node> n = manager().model()->findOrCreate<GroupingNode>(node()->id());
        // each item needs to be an only child
        // this should be changed if multiple NodeItemViews are not used any more
        boost::shared_ptr<gui::Node> p = n->findOrCreate<GroupingNode>((unsigned int)n->getChildren().size());
        p->addChild(m_model_node);
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
    hLayout()->activate();
}

void FNodeParamInput::setEditable(bool editable){
    m_model_node->setMutable(editable);
}

bool FNodeParamInput::willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink *){
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
        pv = qVariantToVariant<ParamValue>(value);
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
        case InputSchedType::Must_Be_New: return Required_Param_Input();
        case InputSchedType::May_Be_Old:  return Optional_Param_Input();
        case InputSchedType::Optional:    return Optional_Param_Input();
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Param_Input();
    }
}

void FNodeParamInput::initView(){
    m_view = new NodeTreeView(true);
    m_view->setIndentation(0);
    m_view->setRootIsDecorated(false);
    //m_view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //m_view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    //m_view->setMinimumSize(QSize(60, height_hint));
    //m_view->setMaximumSize(QSize(1200, 600));
    m_view->setModel(manager().itemModel());
    m_view->updateGeometry();
    m_view->setRootIndex(manager().itemModel()->indexFromNode(m_model_node->getParent()));
    m_view->resize(m_view->sizeHint());
    //m_view->setRootIndex(m_model->index(0, 0));
    //m_view->resizeColumnToContents(0);
    //m_view->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
    // !!! TODO: height hint?
    //m_view->resizeRowsToContents();
    m_view->setAutoFillBackground(false);

    
    QPalette transparent_bg_and_base = palette();
    for(int i=0; i < QPalette::NColorGroups; i++){
         QColor color = transparent_bg_and_base.brush(QPalette::ColorGroup(i), QPalette::Window).color();
         color.setAlpha(0);
         transparent_bg_and_base.setBrush(QPalette::ColorGroup(i), QPalette::Window, QBrush(color));

         //color = transparent_bg_and_base.brush(QPalette::ColorGroup(i), QPalette::Base).color(); 
         color = QColor(0xf3,0xf3,0xf3);
         color.setAlpha(0);
         transparent_bg_and_base.setBrush(QPalette::ColorGroup(i), QPalette::Base, QBrush(color));
    }
    m_view->setPalette(transparent_bg_and_base);

    // umm, doesn't play well with editing widgets!
    //m_view->setStyleSheet("QTreeView {background-color: transparent;}");

    // And this seems to cause weird corruption (but looks just right) :(
    //m_view->setStyleSheet("QTreeView {background-color: #f3f3f3; border: 0px; padding: 0px;}");

    // playing around...
    //m_view->setStyleSheet("QTreeView::item { border: 0px; padding: 0px; }");
    //m_view->setStyleSheet("QTreeView::item {background: transparent;}");
    //m_view->setStyleSheet("QTreeView::item {background: qlineargradient(x1: 0, y1: 0, x2: 0, y2: 1, stop: 0 #e7effd, stop: 1 #cbdaf1);}");

    m_view_proxy = new liquid::ProxyWidget();
    addWidget(m_view_proxy);
    m_view_proxy->setWidget(m_view);
    m_view_proxy->setAutoFillBackground(false);    
    m_view_proxy->setPalette(transparent_bg_and_base);

}

