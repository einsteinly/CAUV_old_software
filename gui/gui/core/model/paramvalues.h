/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#ifndef GUI_PARAMVALUES_H
#define GUI_PARAMVALUES_H

#include <model/variants.h>

#include <QVariant>

namespace cauv {
    namespace gui {

    class Node;

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
    template <> boost::shared_ptr<Node> ParamValueToNode::operator()(Colour & ) const;

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
