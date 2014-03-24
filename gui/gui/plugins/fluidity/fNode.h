/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_ELEMENT_F_NODE_H__
#define __CAUV_ELEMENT_F_NODE_H__

#include <map>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <liquid/node.h>

#include "managedElement.h"
#include "types.h"
#include "model.h"

namespace cauv{
namespace gui{
namespace f{

class FNodeInput;
class FNodeParamInput;
class FNodeOutput;
class ImageSource;

class FNode: public liquid::LiquidNode,
             public ManagedElement {
        Q_OBJECT
    friend class GuiNodeModel;
    public:
        FNode(boost::shared_ptr<GuiNodeModel> node, Manager &m);
        ~FNode();
        std::string getName() const { return m_name; };
        boost::shared_ptr<GuiNodeModel> getModel(){ return m_associated_node; };

//         node_id_t id() const{ return m_node_id; }
//         NodeType::e nodeType() const{ return m_type; }

//         void setType(NodeType::e const&);
//         void setInputs(msg_node_input_map_t const&);
//         void setInputLinks(msg_node_input_map_t const&);
//         void setOutputs(msg_node_output_map_t const&);
//         void setOutputLinks(msg_node_output_map_t const&);
//         void setParams(msg_node_param_map_t const&);
//         void setParamLinks(msg_node_input_map_t const& inputs);
//         void connectOutputTo(const std::string& output, fnode_ptr, const std::string& input);
//         void disconnectOutputFrom(const std::string& output, fnode_ptr, const std::string& input);
//         void addImageDisplayOnInput(const std::string& input, boost::shared_ptr<ImageSource>);

        virtual void status(Status const& s, const std::string& status_information="");
        virtual void status(Status const& s, float const& throughput, float const& frequency, float const& time_taken, float const& time_ratio);

    protected:
        void initIO();
        
        void constructArcTo(const std::string output, FNode* to, const std::string input);
        void destructArcFrom(const std::string input, FNode* from, const std::string output);
        FNodeInput* getInput(const std::string input_name);
        FNodeOutput* getOutput(const std::string output_name);
    
    Q_SIGNALS:
        void closed(FNode&);
    
    public:
    // overridden virtual slots (don't need to be marked as slots):
        virtual void close();

    public Q_SLOTS:
        virtual FNode* remove();
        
        virtual void reExec();
        virtual void duplicate();
        virtual void toggleCollapsed();
        virtual void modelParamValueChanged(const std::string& input, QVariant variant);

    protected:
        void initButtons();

    protected:
        bool m_collapsed;
        boost::shared_ptr<GuiNodeModel> m_associated_node;
        std::map<const std::string, FNodeInput*> m_inputs;
        std::map<const std::string, FNodeOutput*> m_outputs;
        const std::string m_name;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_F_NODE_H__
