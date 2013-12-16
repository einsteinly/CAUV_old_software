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

class QGraphicsProxyWidget;

namespace cauv{

namespace gui{

class NodeItemModel;
class NodeTreeView;
class Node;

namespace f{

class FNodeInput: //public pipeline_model::InputModel,
                  public liquid::ArcSinkLabel,
                  public liquid::ConnectionSink,
                  public ManagedElement {
    Q_OBJECT

    public:
        FNodeInput(FNode &node, Manager& m);
        virtual ~FNodeInput();
    protected:

        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);

    protected Q_SLOTS:
        void modelValueChanged(QVariant value);

    public:
        void addWidget(QGraphicsWidget* w);
        void removeWidget(QGraphicsWidget* w);

        virtual void setCollapsed(bool state);

        void setValue(pipeline_model::ParamValue const& v);

        void setEditable(bool editable);

        static liquid::CutoutStyle const& cutoutStyleForParam();

        void initView();

    protected:
        boost::shared_ptr<Node> m_model_node;
        NodeTreeView* m_view;
        liquid::ProxyWidget* m_view_proxy;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


