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

#include <liquid/connectionSink.h>
#include <liquid/requiresCutout.h>
#include <liquid/connectionSink.h>
#include <liquid/forward.h>

#include <QGraphicsWidget>

#include "elements/style.h"

#include "fluidity/managedElement.h"
#include "fluidity/fNode.h"
#include "fluidity/fNodeIO.h"

class QGraphicsProxyWidget;

namespace cauv{

struct LocalNodeInput;

namespace gui{
namespace f{

class FNodeInput: public QGraphicsWidget,
                  public liquid::RequiresCutout,
                  public liquid::ConnectionSink,
                  public FNodeIO,
                  public ManagedElement{
    protected:
        FNodeInput(Manager& m,
                   liquid::ArcStyle const& of_style,
                   liquid::CutoutStyle const& with_cutout,
                   FNode* node,
                   std::string const& text);
        virtual ~FNodeInput();
        
        // RequiresCutout:
        virtual QList<liquid::CutoutStyle> cutoutGeometry() const;
        
        // QGraphicsItem:
        virtual void paint(QPainter *painter,
                           const QStyleOptionGraphicsItem *option,
                           QWidget *widget = 0);
        
        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source);
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source);

    protected:
        liquid::ArcSink* m_arc_sink;
        QGraphicsProxyWidget* m_text;
};


class FNodeImageInput: public FNodeInput{
    public:
        FNodeImageInput(Manager& m, LocalNodeInput const& input, FNode* node); 
        virtual OutputType::e ioType() const;
        virtual SubType subType() const;
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st);
};


class FNodeParamInput: public FNodeInput{
    public:
        FNodeParamInput(Manager& m, LocalNodeInput const& input, FNode* node);
        virtual OutputType::e ioType() const;
        virtual SubType subType() const;
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st);

        SubType m_subtype;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


