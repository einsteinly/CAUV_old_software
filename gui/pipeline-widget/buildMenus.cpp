#include "buildMenus.h"

#include "pipelineWidget.h"
#include "renderable/listMenu.h"

#include <boost/ref.hpp>

#include <utility/string.h>
#include <generated/messages.h>
#include <img-pipeline/nodeFactory.h>

namespace cauv{
namespace pw{

struct SendAddNodeMessage{
    SendAddNodeMessage(pw_ptr_t p, NodeType::e const& id)
        : m_widget(p), m_id(id){
    }

    void operator()(){
        boost::shared_ptr<AddNodeMessage> msg = boost::make_shared<AddNodeMessage>();
        msg->pipelineName(m_widget->pipelineName());
        msg->nodeType(m_id);
        m_widget->send(msg);
    }

    pw_ptr_t m_widget;
    NodeType::e m_id;
};

boost::shared_ptr<Menu> buildAddNodeMenu(pw_ptr_t p){
    std::map<std::string, boost::shared_ptr<SendAddNodeMessage> > menu_items;
    
    debug() << "buildAddNodeMenu";

    /* this only works if nodes.cpp is compiled in... don't want to do that
    std::list<NodeType::e> available = NodeFactoryRegister::list();
    std::list<NodeType::e>::const_iterator i;
    for(i = available.begin(); i != available.end(); i++)
        menu_items[toStr(*i)] = boost::make_shared<SendAddNodeMessage>(p, *i);
    */
    /* instead rely on generated NodeType::NumValues, and NodeTypes being
     * consecutive
     */
    for(int i = 0; i < NodeType::NumValues; i++)
        menu_items[toStr((NodeType::e)i)] =
            boost::make_shared<SendAddNodeMessage>(p, (NodeType::e)i); 
    
    debug() << __func__ << "returning menu with" << menu_items.size() << "items";
    return boost::make_shared< ListMenu<SendAddNodeMessage> >(p, menu_items);
}

} // namespace pw
} // namespace cauv

