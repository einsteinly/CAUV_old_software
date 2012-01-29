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

#ifndef GUI_PARAMVALUES_H
#define GUI_PARAMVALUES_H

#include <gui/core/model/variants.h>

#include <QVariant>

#include <gui/core/model/node.h>

// register param values types as qt meta types
//!!! todo: generate these?
#include <QMetaType>
#include <generated/types/ParamValue.h>
Q_DECLARE_METATYPE(std::basic_string<char>)
Q_DECLARE_METATYPE(std::vector<cauv::Corner>)
Q_DECLARE_METATYPE(std::vector<cauv::Line>)
Q_DECLARE_METATYPE(std::vector<cauv::Circle>)
Q_DECLARE_METATYPE(std::vector<float>)
Q_DECLARE_METATYPE(std::vector<cauv::KeyPoint>)
Q_DECLARE_METATYPE(cauv::BoundedFloat)

namespace cauv {
    namespace gui {

    struct ParamValueToNode : public boost::static_visitor<boost::shared_ptr<Node> >
    {
        ParamValueToNode(const nid_t id, boost::shared_ptr<Node> parent);

        template <typename T> boost::shared_ptr<Node> operator()( T & ) const
        {
            throw std::runtime_error("Unsupported ParamValue type");
        }

        nid_t m_id;
        boost::shared_ptr<Node> m_parent;
    };

    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(int &) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(float & ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(std::string & ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(bool & operand ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(BoundedFloat & ) const;

    template <class T>
    boost::shared_ptr<Node> paramValueToNode(nid_t id, boost::shared_ptr<Node> parent, T boostVariant){
        return boost::apply_visitor(ParamValueToNode(id, parent), boostVariant);
    }



    template<class T>
    std::map<std::string, ParamValue> nodeListToParamValueMap(const std::vector<boost::shared_ptr<T> > nodes){
        std::map<std::string, ParamValue> values;
        foreach(boost::shared_ptr<T> const& node, nodes) {
            try {
                values[boost::get<std::string>(node->nodeId())] = qVariantToVariant<ParamValue>(node->get());
            } catch (std::bad_cast){
                error() << "Failed while converting QVariant to Variant for " << node->nodePath();
                continue;
            }
        }
        return values;
    }

    } // namespace gui
} // namespace cauv


#endif // GUI_PARAMVALUES_H
