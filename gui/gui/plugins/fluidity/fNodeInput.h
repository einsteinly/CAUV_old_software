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

#include <fluidity/managedElement.h>
#include <fluidity/fNode.h>

class QGraphicsProxyWidget;

namespace cauv{

namespace gui{

class NodeItemModel;
class NodeTreeView;
class Node;

namespace f{

class FNodeInput: public liquid::ArcSinkLabel,
                  public liquid::ConnectionSink,
                  public ManagedElement {
    Q_OBJECT

    public:
        FNodeInput(const std::string input_name, FNode &node, Manager& m);
        virtual ~FNodeInput();
    protected:

        // ConnectionSink:
        virtual bool willAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);
        virtual ConnectionStatus doAcceptConnection(liquid::ArcSourceDelegate* from_source, liquid::AbstractArcSink*);
        
        pipeline_model::InputModel& getModel();

    Q_SIGNALS:
        void modelValueChanged(const std::string& input, QVariant value);
        
    public Q_SLOTS:
        void widgetValueChanged(QVariant value);

    public:
        void addWidget(QGraphicsWidget* w);
        void removeWidget(QGraphicsWidget* w);

        virtual void setCollapsed(bool state);

        void setValue(boost::shared_ptr<pipeline_model::ParamValue> const& v);

        void setEditable(bool editable);

        static liquid::CutoutStyle const& cutoutStyleForParam();

        void initView();

    protected:
        boost::shared_ptr<Node> m_model_node; //gui node for actual data
        NodeTreeView* m_view;
        liquid::ProxyWidget* m_view_proxy;
        std::string m_input_name;
        FNode* m_node;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_INPUT_H__


