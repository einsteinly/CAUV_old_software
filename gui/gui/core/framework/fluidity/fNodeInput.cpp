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
    : ArcSinkLabel(new liquid::ArcSink(of_style, with_cutout, this),
                   node,
                   QString::fromStdString(id)),
      FNodeIO(node, id),
      ManagedElement(m){
}

FNodeInput::~FNodeInput(){
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


