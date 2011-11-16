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

#ifndef __CAUV_GUI_FNODE_INPUT_H__
#define __CAUV_GUI_FNODE_INPUT_H__

#include <QGraphicsWidget>
#include <QGraphicsLinearLayout>
#include <QPainter>

#include <liquid/arcSink.h>
#include <liquid/connectionSink.h>

#include "elements/style.h"

#include "fNodeIO.h"

namespace cauv{
namespace gui{
namespace f{

class FNodeInput: public liquid::ArcSink,
                  public FNodeIO{
    public:
        FNodeInput(liquid::ArcStyle const& of_style,
                   liquid::CutoutStyle const& with_cutout,
                   FNode* node)
            : liquid::ArcSink(of_style, with_cutout, this),
              FNodeIO(node){
        }
        virtual ~FNodeInput(){}

        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source){
            FNodeOutput* output = dynamic_cast<FNodeOutput*>(from_source);
            debug() << "fNodeInput::willAcceptConnection from_source=" << from_source << output;
            if(output)
                return output->ioType() == ioType() &&
                       output->subType() == subType() &&
                       output->node() != node();
            return false;
        }
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source){
            debug() << "FNodeInput::doAcceptConnection:" << dynamic_cast<FNodeOutput*>(from_source);
            return Rejected;
        }
};



class FNodeImageInput: public FNodeInput{
    public:
        FNodeImageInput(InputSchedType::e const& sched_type, FNode* node)
            : FNodeInput(Image_Arc_Style, cutoutStyleForSchedType(sched_type), node){
        }

        virtual OutputType::e ioType() const{
            return OutputType::Image;
        }

        virtual SubType subType() const{
            return -1;
        }
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st){
            switch(st){
                case InputSchedType::Must_Be_New: return Required_Image_Input;
                case InputSchedType::May_Be_Old: return Optional_Image_Input;
                default:
                    error() << "unknown InputSchedtype:" << st;
                    return Required_Image_Input;
            }
        }
};

class FNodeParamInput: public FNodeInput{
    public:
        FNodeParamInput(InputSchedType::e const& sched_type,
                        SubType const& subtype,
                        FNode* node)
            : FNodeInput(Image_Arc_Style, cutoutStyleForSchedType(sched_type), node), m_subtype(subtype){
        }

        virtual OutputType::e ioType() const{
            return OutputType::Parameter;
        }

        virtual SubType subType() const{
            return m_subtype;
        }
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st){
            switch(st){
                case InputSchedType::Must_Be_New: return Required_Param_Input;
                case InputSchedType::May_Be_Old: return Optional_Param_Input;
                default:
                    error() << "unknown InputSchedtype:" << st;
                    return Required_Image_Input;
            }
        }

        SubType m_subtype;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


