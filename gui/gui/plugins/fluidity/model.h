/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_F_PIPELINE_MODEL__
#define __CAUV_GUI_F_PIPELINE_MODEL__

#include <boost/shared_ptr.hpp>

#include <pipeline_model/pipeline.h>
#include <pipeline_model/node_model.h>
#include <pipeline_model/edge_model.h>

namespace cauv{
namespace gui{
namespace f{

class FNode;
class Manager;
class GuiPipelineModel;

//ensures gui is updated when underlying model is updated
class GuiNodeModel: public pipeline_model::NodeModel {
    public:
        GuiNodeModel(const std::string type, GuiPipelineModel &pipeline);
        GuiNodeModel(const pipeline_model::NodeModelType& type_, GuiPipelineModel &pipeline_);
        
        void setFNode(FNode* node);
        FNode* getFNode();
        
        virtual void connectOutput(const std::string output, pipeline_model::InputModel& input);
        //virtual void connectInput(const std::string input, OutputModel&);
        
        virtual void disconnectOutput(const std::string output);
        virtual void disconnectInput(const std::string input);
    private:
        FNode* m_node;
};

class GuiPipelineModel: public pipeline_model::PipelineModel {
    public:
        GuiPipelineModel(const std::string& pipeline_name, Manager* manager);
        virtual boost::shared_ptr<pipeline_model::NodeModel> addNode(const std::string type);
        virtual void delNode(const std::string &name);
    
    protected:
        virtual boost::shared_ptr<pipeline_model::NodeModel> constructNode(const std::string type);
    
    private:
        Manager* m_manager;
};

}
}
}

#endif
