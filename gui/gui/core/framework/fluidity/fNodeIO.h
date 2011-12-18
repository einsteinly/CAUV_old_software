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

#ifndef __CAUV_GUI_FNODE_IO_H__
#define __CAUV_GUI_FNODE_IO_H__

#include <generated/types/OutputType.h>

namespace cauv{
namespace gui{
namespace f{

class FNode;

// !!! corresponds to messages.msg
typedef int32_t SubType;

class FNodeIO{
    public:
        FNodeIO(FNode* node, std::string const& id)
            : m_node(node), m_id(id){
        }

        FNode* node() const{
            return m_node;
        }

        std::string const& id() const{ return m_id; }
        
        virtual SubType subType() const = 0;
        virtual OutputType::e ioType() const = 0;

    protected:
        FNode* m_node;
        std::string m_id;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_GUI_FNODE_IO_H__

