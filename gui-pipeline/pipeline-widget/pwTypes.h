#ifndef __PW_TYPES_H__
#define __PW_TYPES_H__

/* forward declarations and pointer typedefs for all PipelineWidget and
 * Renderable types
 */

#include <boost/shared_ptr.hpp>

namespace pw{

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
class NodeOutputBlob;
class FloatingArcHandle;


typedef PipelineWidget* pw_ptr_t;
typedef Container* container_ptr_t;

typedef boost::weak_ptr<Renderable> renderable_wkptr_t;
typedef boost::shared_ptr<Renderable> renderable_ptr_t;
typedef boost::shared_ptr<Node> node_ptr_t;
typedef boost::shared_ptr<Arc> arc_ptr_t;
typedef boost::shared_ptr<Menu> menu_ptr_t;
typedef boost::shared_ptr<Text> text_ptr_t;

// TODO: this should really be synchronised with pipelineTypes.h! (need
// a pipeline namespace to do that without confusion about Nodes though)
typedef int32_t node_id;

} // namespace pw

#endif // ndef __PW_TYPES_H__

