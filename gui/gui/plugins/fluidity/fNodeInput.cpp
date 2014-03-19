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
//#include <model/variants.h>
#include <model/nodes/numericnode.h>
#include <model/nodes/groupingnode.h>
#include <model/nodes/stringnode.h>
#include <model/nodes/colournode.h>
#include <model/nodeItemModel.h>
#include <nodepicker.h>

#include "manager.h"
#include "fNodeOutput.h"

using namespace cauv;
using namespace cauv::gui::f;

// - FNodeInput
FNodeInput::FNodeInput(const std::string input_name, FNode &node, Manager &m)
    : ArcSinkLabel(new liquid::ArcSink(Param_Arc_Style(), Optional_Param_Input(), this),
                   &node,
                   QString::fromStdString(input_name)),
      ManagedElement(m),
      m_input_name(input_name),
      m_node(&node){
    // don't want a label:
    //removeWidget(m_text);
    //m_text->deleteLater();
    //m_text = nullptr;
    CAUV_LOG_DEBUG(2, "Created input " << m_input_name << " for FNode " << node.getName());
}

FNodeInput::~FNodeInput(){
    // if it isn't in the layout, then need to delete the view proxy
#warning TODO: figure out if this is still needed, and figure out why it segfaults
    /*
    for(int i = 0; i < hLayout()->count(); i++)
        if(hLayout()->itemAt(i) == m_view_proxy)
            return;
    m_view_proxy->deleteLater();
    */
    CAUV_LOG_DEBUG(2, "Destroyed input " << m_input_name);
}

pipeline_model::InputModel& FNodeInput::getModel(){
    return m_node->getModel()->getInput(m_input_name);
};

void FNodeInput::setCollapsed(bool collapsed){
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

void FNodeInput::addWidget(QGraphicsWidget* w){
    w->setParent(nullptr);
    hLayout()->addItem(w);
}

void FNodeInput::removeWidget(QGraphicsWidget* w){
    hLayout()->removeItem(w);
}

bool FNodeInput::willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
    //check input type in pipeline model
    return getModel().canAccept(output->getModel());
}


FNodeInput::ConnectionStatus FNodeInput::doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*){
    FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
     if (manager().createArc(output->getNode(), output->getName(), getModel())){
        return Accepted;
     } else {
        return Rejected;
     }
}

#if 0
// - static helper stuff:
struct MakeModelNode: boost::static_visitor<boost::shared_ptr<cauv::gui::Node> >{
    MakeModelNode(const std::string& id) : id(id) { }
    
    // catch-all for uneditable types...
    template<typename T>
    boost::shared_ptr<cauv::gui::Node> operator()(T const&) const{
        CAUV_LOG_DEBUG(7, "MakeModelNode: catch all (string)");    
        return boost::make_shared< cauv::gui::StringNode >(cauv::gui::nid_t(id));
    }

    boost::shared_ptr<cauv::gui::Node> operator()(float const&) const{
        CAUV_LOG_DEBUG(7, "MakeModelNode: float");
        typedef cauv::gui::NumericNode<float> float_node_t;
        boost::shared_ptr<float_node_t> r = boost::make_shared<float_node_t>(cauv::gui::nid_t(id));
        r->setPrecision(8);
        return r;
    }
    
    boost::shared_ptr<cauv::gui::Node> operator()(bool const&) const{
        CAUV_LOG_DEBUG(7, "MakeModelNode: bool");    
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

    boost::shared_ptr<cauv::gui::Node> operator()(const std::string&) const{
        debug(7) << "MakeModelNode: string";    
        // !!! need a non-editable node
        return boost::make_shared<cauv::gui::StringNode>(cauv::gui::nid_t(id));
    }

    private:
        const std::string& id;
};

static boost::shared_ptr<cauv::gui::Node> makeModelNodeForInput(const std::string& id, pipeline_model::ParamValue const& v){
    return boost::apply_visitor(MakeModelNode(id), v);
    //return boost::shared_ptr<cauv::gui::Node>();
}

#endif

void FNodeInput::setValue(pipeline_model::ParamValue const& v){
#if 0
    if(!m_model_node){
        m_model_node = makeModelNodeForInput(m_input_name, v);
        boost::shared_ptr<gui::Node> n = manager().model()->findOrCreate<GroupingNode>(m_node->getName());
        // each item needs to be an only child
        // this should be changed if multiple NodeItemViews are not used any more
        boost::shared_ptr<gui::Node> p = n->findOrCreate<GroupingNode>((unsigned int)n->getChildren().size());
        p->addChild(m_model_node);
        connect(m_model_node.get(), SIGNAL(onSet(QVariant)), this, SLOT(modelValueChanged(QVariant)));
        initView();
    }
    //m_model_node->update(paramValueToQVariant(v));
#endif
}

void FNodeInput::setEditable(bool editable){
    m_model_node->setMutable(editable);
}

void FNodeInput::modelValueChanged(QVariant value){
}

liquid::CutoutStyle const& FNodeInput::cutoutStyleForParam(){
    return Optional_Param_Input();
}

void FNodeInput::initView(){
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
