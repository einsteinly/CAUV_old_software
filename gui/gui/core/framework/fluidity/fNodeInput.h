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
#include <liquid/arcSinkLabel.h>

#include "elements/style.h"

#include "fluidity/managedElement.h"
#include "fluidity/fNode.h"
#include "fluidity/fNodeIO.h"

class QGraphicsProxyWidget;

namespace cauv{

struct LocalNodeInput;

namespace gui{

class SingleNodeItemModel;
class NodeTreeView;
class Node;

namespace f{

class FNodeInput: public liquid::ArcSinkLabel,
                  public liquid::ConnectionSink,
                  public FNodeIO,
                  public ManagedElement{
    protected:
        FNodeInput(Manager& m,
                   liquid::ArcStyle const& of_style,
                   liquid::CutoutStyle const& with_cutout,
                   FNode* node,
                   std::string const& id);
        virtual ~FNodeInput();

        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source);
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source);

    public:
        void addWidget(QGraphicsWidget* w);
        void removeWidget(QGraphicsWidget* w);

        virtual void setCollapsed(bool state);
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
    Q_OBJECT
    public:
        FNodeParamInput(Manager& m, LocalNodeInput const& input, FNode* node);
        ~FNodeParamInput();
        virtual OutputType::e ioType() const;
        virtual SubType subType() const;

        virtual void setCollapsed(bool state);

        void setValue(ParamValue const& v);

        void setEditable(bool editable);

        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source);

    protected Q_SLOTS:
        void modelValueChanged(QVariant value);
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st);

        void initView();

        SubType m_subtype;
        std::set<int32_t> m_compatible_subtypes;
        
        SingleNodeItemModel* m_model;
        boost::shared_ptr<Node> m_model_node;
        NodeTreeView* m_view;
        QGraphicsProxyWidget* m_view_proxy;
        
        // !!! workaround, should only exist in the model, see implementation
        // of modelValueChanged
        ParamValue m_value;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


