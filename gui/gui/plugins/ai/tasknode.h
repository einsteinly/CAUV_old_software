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

#ifndef __CAUV_AI_TASK_NODE_H__
#define __CAUV_AI_TASK_NODE_H__

#include <boost/shared_ptr.hpp>

#include <vector>

#include <gui/core/model/nodes/numericnode.h>

namespace cauv {
namespace gui {


GENERATE_SIMPLE_NODE(AiMissionNode)
GENERATE_SIMPLE_NODE(AiConditionNode)
GENERATE_SIMPLE_NODE(PipelineNode) //!!! todo: this shouldn't be deifned here

class AiTaskNode : public Node {
    public:
        AiTaskNode(const nid_t id) : Node(id, nodeType<AiTaskNode>()){
        }

        void addCondition(boost::shared_ptr<AiConditionNode> condition){
            if(std::find(m_conditions.begin(), m_conditions.end(), condition) == m_conditions.end())
                m_conditions.push_back(condition);
        }

        void removeCondition(boost::shared_ptr<AiConditionNode> condition){
            m_conditions.erase(std::find(m_conditions.begin(), m_conditions.end(), condition));
        }

        std::vector<boost::shared_ptr<AiConditionNode> > getConditions(){
            return m_conditions;
        }



        boost::shared_ptr<Node> setDebug(std::string name, ParamValue value){
            if (!m_debug[name]) {
                m_debug[name] = paramValueToNode(nid_t(name), value);
                addChild(m_debug[name]);
            }
            m_debug[name]->update(variantToQVariant(value));
            return m_debug[name];
        }

        void removeDebug(std::string name){
            this->removeChild(nid_t(name));
            m_debug.erase(name);
        }

        std::map<std::string, boost::shared_ptr<Node> > getDebugValues(){
            return m_debug;
        }



        boost::shared_ptr<Node> setStaticOption(std::string name, ParamValue value){
            if (!m_staticOptions[name]) {
                m_staticOptions[name] = paramValueToNode(nid_t(name), value);
                addChild(m_staticOptions[name]);
            }
            m_staticOptions[name]->update(variantToQVariant(value));
            return m_staticOptions[name];
        }

        void removeStaticOption(std::string name){
            this->removeChild(nid_t(name));
            m_staticOptions.erase(name);
        }

        std::map<std::string, boost::shared_ptr<Node> > getStaticOptions(){
            return m_staticOptions;
        }



        boost::shared_ptr<Node> setDynamicOption(std::string name, ParamValue value){
            if (!m_dynamicOptions[name]) {
                m_dynamicOptions[name] = paramValueToNode(nid_t(name), value);
                addChild(m_dynamicOptions[name]);
            }
            m_dynamicOptions[name]->update(variantToQVariant(value));
            return m_dynamicOptions[name];
        }

        void removeDynamicOption(std::string name){
            this->removeChild(name);
            m_dynamicOptions.erase(name);
        }

        std::map<std::string, boost::shared_ptr<Node> > getDynamicOptions(){
            return m_dynamicOptions;
        }



        boost::shared_ptr<Node> setTaskOption(std::string name, ParamValue value){
            if (!m_taskOptions[name]) {
                m_taskOptions[name] = paramValueToNode(nid_t(name), value);
                addChild(m_taskOptions[name]);
            }
            m_taskOptions[name]->update(variantToQVariant(value));
            return m_taskOptions[name];
        }

        void removeTaskOption(std::string name){
            this->removeChild(nid_t(name));
            m_taskOptions.erase(name);
        }

        std::map<std::string, boost::shared_ptr<Node> > getTaskOptions(){
            return m_taskOptions;
        }

    protected:
        std::vector<boost::shared_ptr<AiConditionNode> > m_conditions;
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_staticOptions;
        std::map<std::string, boost::shared_ptr<Node> > m_dynamicOptions;
        std::map<std::string, boost::shared_ptr<Node> > m_taskOptions;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_TASK_NODE_H__
