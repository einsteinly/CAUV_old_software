#ifndef __CAUV_ELEMENT_NODE_INPUT_H__
#define __CAUV_ELEMENT_NODE_INPUT_H__

#include <QGraphicsPathItem>

namespace cauv{
namespace gui{

// !!! synchronise with pipeline
namespace NodeIOType{
enum e{
    Image, Parameter
};
}// namespace NodeIOType

struct NodeStyle;

class NodeInput: public QGraphicsPathItem{
    public:
        NodeInput(NodeStyle const& style, NodeIOType::e const& type,
                  bool required, QGraphicsItem *parent=NULL);

    protected:
        NodeIOType::e m_type;
        bool m_required;
        NodeStyle const& m_style;
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_ELEMENT_NODE_INPUT_H__

