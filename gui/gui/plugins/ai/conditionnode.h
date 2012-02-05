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

#ifndef __CAUV_AI_CONDITION_NODE_H__
#define __CAUV_AI_CONDITION_NODE_H__

#include <boost/shared_ptr.hpp>

#include <generated/types/ParamValue.h>

#include <gui/core/model/node.h>

namespace cauv {
namespace gui {


class AiConditionNode : public Node {
    public:
        AiConditionNode(const nid_t id);

        boost::shared_ptr<Node> setDebug(std::string name, ParamValue value);
        void removeDebug(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getDebugValues();

        boost::shared_ptr<Node> setOption(std::string name, ParamValue value);
        void removeOption(std::string name);
        std::map<std::string, boost::shared_ptr<Node> > getOptions();

        static void addType(std::string type);
        static std::set<std::string> getTypes();

    protected:
        std::map<std::string, boost::shared_ptr<Node> > m_debug;
        std::map<std::string, boost::shared_ptr<Node> > m_options;

        static std::set<std::string> m_types;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_AI_CONDITION_NODE_H__
