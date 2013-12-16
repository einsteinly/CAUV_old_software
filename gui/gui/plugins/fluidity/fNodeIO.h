/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef __CAUV_GUI_FNODE_IO_H__
#define __CAUV_GUI_FNODE_IO_H__

namespace cauv{
namespace gui{
namespace f{
    
#include <common/pipeline_model/edge_model.h>

class FNode;

class FNodeIO {
    public:
        FNodeIO(FNode* node, const std::string& id)
            : m_node(node), m_id(id){
        }

        FNode* node() const{
            return m_node;
        }

        const std::string& id() const{ return m_id; }
    protected:
        FNode* m_node;
        std::string m_id;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_IO_H__

