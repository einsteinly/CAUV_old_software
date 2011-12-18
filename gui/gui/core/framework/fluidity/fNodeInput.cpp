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

#include <liquid/arcSink.h>

#include <QGraphicsProxyWidget>
#include <QLabel>
#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>

#include <generated/types/LocalNodeInput.h>

#include <debug/cauv_debug.h>

#include "fluidity/manager.h"
#include "fluidity/fNodeOutput.h"

using namespace cauv;
using namespace cauv::gui::f;

// - FNodeInput
FNodeInput::FNodeInput(Manager& m,
                       liquid::ArcStyle const& of_style,
                       liquid::CutoutStyle const& with_cutout,
                       FNode* node,
                       std::string const& id)
    : QGraphicsWidget(node),
      RequiresCutout(),
      FNodeIO(node, id),
      ManagedElement(m),
      m_arc_sink(new liquid::ArcSink(of_style, with_cutout, this)),
      m_text(NULL){

    QGraphicsLinearLayout *hlayout = new QGraphicsLinearLayout(
        Qt::Horizontal, this
    );
    hlayout->setSpacing(0);
    hlayout->setContentsMargins(0,0,0,0);

    m_arc_sink->setParentItem(this);
    hlayout->addItem(m_arc_sink);
    hlayout->setAlignment(m_arc_sink, Qt::AlignVCenter | Qt::AlignLeft);

    QLabel* text_label = new QLabel(QString::fromStdString(id));
    text_label->setTextInteractionFlags(Qt::NoTextInteraction);
    text_label->setFont(F_Node_Style.text.font);
    m_text = new QGraphicsProxyWidget();
    m_text->setWidget(text_label);
    hlayout->addItem(m_text);
    hlayout->setAlignment(m_text, Qt::AlignVCenter | Qt::AlignLeft);

    hlayout->setItemSpacing(1, 4.0); 
    
    hlayout->addStretch(1);

    setLayout(hlayout);
     
    connect(node, SIGNAL(xChanged()), m_arc_sink, SIGNAL(geometryChanged()));
    connect(node, SIGNAL(yChanged()), m_arc_sink, SIGNAL(geometryChanged()));
}

FNodeInput::~FNodeInput(){
}

QList<liquid::CutoutStyle> FNodeInput::cutoutGeometry() const{
    QList<liquid::CutoutStyle> sink_cutouts =  m_arc_sink->cutoutGeometry();
    QList<liquid::CutoutStyle> r;
    foreach(liquid::CutoutStyle const& c, sink_cutouts){
        liquid::CutoutStyle t = c;
        t.main_cutout.y_offset += m_arc_sink->pos().y();
        t.second_cutout.y_offset += m_arc_sink->pos().y();
        r << t;
    }
    return r;
}


void FNodeInput::paint(QPainter *painter,
                       const QStyleOptionGraphicsItem *option,
                       QWidget *widget){
    Q_UNUSED(option);
    Q_UNUSED(widget);
    painter->setPen(QPen(QColor(20,180,40,64)));
    painter->setBrush(Qt::NoBrush);
    painter->drawRect(boundingRect());
    debug(11) << "arc sink is at:"
              << m_arc_sink->pos().x()
              << m_arc_sink->pos().y()
              << m_arc_sink->geometry().width()
              << m_arc_sink->geometry().height()
              << m_arc_sink->parentItem();
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
       output->subType() == subType() &&
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
        case InputSchedType::May_Be_Old: return Optional_Image_Input;
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Image_Input;
    }
}

// - FNodeParamInput
FNodeParamInput::FNodeParamInput(Manager& m, LocalNodeInput const& input, FNode* node)
    : FNodeInput(m, Image_Arc_Style, cutoutStyleForSchedType(input.schedType), node, input.input),
      m_subtype(input.subType){
}

OutputType::e FNodeParamInput::ioType() const{
    return OutputType::Parameter;
}

SubType FNodeParamInput::subType() const{
    return m_subtype;
}

liquid::CutoutStyle const& FNodeParamInput::cutoutStyleForSchedType(InputSchedType::e const& st){
    switch(st){
        case InputSchedType::Must_Be_New: return Required_Param_Input;
        case InputSchedType::May_Be_Old: return Optional_Param_Input;
        default:
            error() << "unknown InputSchedtype:" << st;
            return Required_Image_Input;
    }
}


