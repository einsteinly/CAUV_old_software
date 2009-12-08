#include "node.h"


class FileInputNode: public Node{
	public:
		FileInputNode(std::vector<node_id, input_id> children)
			: // base constructor?
			{

		}

		void exec(){
			// A) check if any children need a new image (child->isWaiting() == true)
			// B) check if any children are still using any of this node's outputs // TODO: do reference counting pointers do this for us? I think they might
			// if A) is not true, return immediately
			// if B) is true, move aside (copy if necessary? - don't think it is... can probably just delete refcounting pointers to outputs)
			// make new outputs
			// TODO: return a list of children that MAY now be able to run (may, since they may be waiting on other parents)
		}

};

