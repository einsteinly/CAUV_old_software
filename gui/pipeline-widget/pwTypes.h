#ifndef __PW_TYPES_H__
#define __PW_TYPES_H__

/* enum definitions, forward declarations and pointer typedefs for all
 * PipelineWidget and Renderable types
 */

#include <boost/weak_ptr.hpp>
#include <boost/shared_ptr.hpp>

namespace pw{

namespace drawtype_e{
enum e{
    no_flags = 0x0,
    picking  = 0x1
};
} // namespace drawtype_e

class PipelineWidget;
class PipelineGuiCauvNode;

class Container;

class Renderable;
class Node;
class Menu;
class Arc;
class Text;
class NodeIOBlob;
class NodeInputBlob;
class NodeInputParamBlob;
class NodeOutputBlob;
class FloatingArcHandle;
class ImgNode;
class PVPairEditableBase;
template<typename value_T> class PVPair;


typedef PipelineWidget* pw_ptr_t;
typedef Container* container_ptr_t;

typedef boost::weak_ptr<Renderable> renderable_wkptr_t;
typedef boost::shared_ptr<Renderable> renderable_ptr_t;
typedef boost::weak_ptr<Node> node_wkptr_t;
typedef boost::shared_ptr<Node> node_ptr_t;
typedef boost::shared_ptr<Arc> arc_ptr_t;
typedef boost::shared_ptr<Menu> menu_ptr_t;
typedef boost::shared_ptr<Text> text_ptr_t;
typedef boost::shared_ptr<ImgNode> imgnode_ptr_t;

// TODO: this should really be synchronised with pipelineTypes.h! (need
// a pipeline namespace to do that without confusion about Nodes though)
typedef int32_t node_id;

// Types for OverKey:
namespace ok{

namespace keystate_e{
enum e{
    released = 0x0,
    pressed = 0x1,
    num_values = 0x2
};
} // namespace keystate_e

class Action;
class Key;
class OverKey;

typedef boost::shared_ptr<Action> action_ptr_t;
typedef boost::shared_ptr<Key> key_ptr_t;
typedef boost::shared_ptr<OverKey> overlay_ptr_t;

} // namespace ok

} // namespace pw

#endif // ndef __PW_TYPES_H__

