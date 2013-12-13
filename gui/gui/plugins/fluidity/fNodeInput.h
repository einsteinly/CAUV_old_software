/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_FNODE_INPUT_H__
#define __CAUV_GUI_FNODE_INPUT_H__

#include <liquid/connectionSink.h>
#include <liquid/requiresCutout.h>
#include <liquid/connectionSink.h>
#include <liquid/forward.h>
#include <liquid/arcSinkLabel.h>
#include <liquid/proxyWidget.h>

#include <elements/style.h>

#include <common/pipeline_model/edge_model.h>

#include <fluidity/managedElement.h>
#include <fluidity/fNode.h>
#include <fluidity/fNodeIO.h>

class QGraphicsProxyWidget;

namespace cauv{

struct LocalNodeInput;

namespace gui{

class NodeItemModel;
class NodeTreeView;
class Node;

namespace f{

class FNodeInput: public pipeline_model::InputModel,
                  public liquid::ArcSinkLabel,
                  public liquid::ConnectionSink,
                  public FNodeIO,
                  public ManagedElement{
    protected:
        FNodeInput(Manager& m,
                   liquid::ArcStyle const& of_style,
                   liquid::CutoutStyle const& with_cutout,
                   FNode* node,
                   const std::string& id);
        virtual ~FNodeInput();

        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);

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
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink* to_sink);

    protected Q_SLOTS:
        void modelValueChanged(QVariant value);
    
    private:
        static liquid::CutoutStyle const& cutoutStyleForSchedType(InputSchedType::e const& st);

        void initView();

        SubType m_subtype;
        std::set<int32_t> m_compatible_subtypes;

        boost::shared_ptr<Node> m_model_node;
        NodeTreeView* m_view;
        liquid::ProxyWidget* m_view_proxy;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


