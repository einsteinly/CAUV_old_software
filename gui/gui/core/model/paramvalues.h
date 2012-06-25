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

#include <map>

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
Q_DECLARE_METATYPE(std::vector<int32_t>)
Q_DECLARE_METATYPE(cauv::ImageRegion)
Q_DECLARE_METATYPE(cauv::Range)
Q_DECLARE_METATYPE(cauv::floatXY)
Q_DECLARE_METATYPE(cauv::DynamicEnum)
Q_DECLARE_METATYPE(std::vector<cauv::floatXY>)
Q_DECLARE_METATYPE(std::vector<cauv::Ellipse>)
Q_DECLARE_METATYPE(cauv::LocationSequence)

namespace cauv {
    namespace gui {

    struct ParamValueToNode : public boost::static_visitor<boost::shared_ptr<Node> >
    {
        ParamValueToNode(const nid_t id);

        template <typename T> boost::shared_ptr<Node> operator()( T & ) const
        {
            throw std::runtime_error("Unsupported ParamValue type");
        }

        nid_t m_id;
    };

    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(int &) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(float & ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(std::string & ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(bool & operand ) const;
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(BoundedFloat & ) const;

    template <class T>
    boost::shared_ptr<Node> paramValueToNode(nid_t id, T boostVariant){
        return boost::apply_visitor(ParamValueToNode(id), boostVariant);
    }



    template<class T>
    std::map<std::string, ParamValue> nodeMapToParamValueMap(const std::map<std::string, boost::shared_ptr<T> > nodes){
        std::map<std::string, ParamValue> values;
        typedef std::map<std::string, boost::shared_ptr<T> > param_map;
        for (typename param_map::const_iterator it = nodes.begin(); it != nodes.end(); ++it) {
            try {
                QVariant v = it->second->get();
                if ((unsigned)v.type() == (unsigned)qMetaTypeId<QString>())
                    v = QVariant::fromValue(v.value<QString>().toStdString());
                values[boost::get<std::string>(it->second->nodeId())] = qVariantToVariant<ParamValue>(v);
            } catch (std::bad_cast){
                error() << "Failed while converting QVariant to Variant for " << it->second->nodePath();
                continue;
            }
        }
        return values;
    }

    } // namespace gui
} // namespace cauv


#endif // GUI_PARAMVALUES_H
