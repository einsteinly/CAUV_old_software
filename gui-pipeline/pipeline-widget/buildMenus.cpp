#include "buildMenus.h"

#include "pipelineWidget.h"
#include "renderable/menu.h"

#include <boost/ref.hpp>

#include <common/cauv_utils.h>
#include <common/messages.h>
#include <img-pipeline/nodeFactory.h>

struct SendAddNodeMessage{
    SendAddNodeMessage(PipelineWidget& p, NodeType::e const& id)
        : m_widget(p), m_id(id){
    }

    void operator()(){
        boost::shared_ptr<AddNodeMessage> msg = boost::make_shared<AddNodeMessage>();
        msg->nodeType(m_id);
        m_widget.sendMessage(msg);
    }

    PipelineWidget& m_widget;
    NodeType::e m_id;
};

boost::shared_ptr<Renderable> buildAddNodeMenu(PipelineWidget& p){
    std::map<std::string, boost::shared_ptr<SendAddNodeMessage> > menu_items;
    
    /* this only works if nodes.cpp is compiled in... don't want to do that
    std::list<NodeType::e> available = NodeFactoryRegister::list();
    std::list<NodeType::e>::const_iterator i;
    for(i = available.begin(); i != available.end(); i++)
        menu_items[to_string(*i)] = boost::make_shared<SendAddNodeMessage>(boost::ref(p), *i);
    */
    /* instead rely on generated NodeType::NumValues, and NodeTypes being
     * consecutive
     */
    for(int i = 0; i < NodeType::NumValues; i++)
        menu_items[to_string((NodeType::e)i)] =
            boost::make_shared<SendAddNodeMessage>(boost::ref(p), (NodeType::e)i); 
    
    debug() << __func__ << "returning menu with" << menu_items.size() << "items";
    return boost::make_shared< Menu<SendAddNodeMessage> >(boost::ref(p), menu_items);
}


