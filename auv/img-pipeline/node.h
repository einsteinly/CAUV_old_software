
#include <map>
#include <boost/tuple/tuple.hpp>
#include <boost/shared_ptr.hpp>

#include "pipeline_types.h"
#include "cv.h"

class Node{
	public:
		Node(Scheduler*){
		}

		void exec(){
			// if this is a fast node: request new image from parents now (demandNewOutput)
			//  
			//  this->doWork();
			//
			// if this is a slow node, request a new image from parents now
			// for each child that takes our output, set a flag saying 'this output is new'
			// 	ie
			// 	for node in children
			// 		node->setNew(node->input[our_output]) // or similar
		}
		
		void newInput(input_id){
			// keeps a record of which inputs are new
			// if all inputs are now new, tell scheduler to schedule us with some priority
		}

		void demandNewOutput(/*output_id ?*/){
			// add self to scheduler with some priority
		}

	protected:
		virtual void doWork() = 0;

	private: 
		void getLock();
		void releaseLock();

		std::map<input_id, boost::tuple<node_id, output_id> > parent_links;
		std::map<output_id, boost::tuple<node_id, input_id> > child_links;
		std::map<output_id, boost::shared_pointer<image> > outputs; // exec() may allocate new outputs, or take a shallow copy of a parent's output 
		
		std::map<input_id, bool> new_inputs;
		

		// TODO: how do we know if an output is still being used? - can we use the reference count of shared_pointer? - probably not
};


class FileInputNode: public Node{
	public:
		virtual void exec(){
		}
};

