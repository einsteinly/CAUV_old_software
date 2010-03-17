#include <string>
#include <iostream>
#include <exception>
#include <stdexcept>

#include <common/cauv_node.h>
#include <common/messages.h>
#include <common/debug.h>

#include <common/spread/cauv_mailbox_monitor.h>


class ImagePipelineTesterNode : public CauvNode{
    public:
        ImagePipelineTesterNode(std::string const& group)
            : CauvNode("pipe-test", group){
        }
    protected:
        virtual void onRun(){
            info() << "--- ImagePipelineTesterNode::onRun ---";
            
            info() << "Add TestMBObserver...";
            eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
            
            info() << "Joining pipeline group...";
            mailbox()->joinGroup("pipeline");
            
            info() << "--- start test sequence ---";
            /* test message sequence */
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;

            AddNodeMessage an(nt_file_input, arcs_in, arcs_out);
            
            // Add input node
            info() << "Add file input node:";
            info() << "\t" << an;
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            

            // Add output node
            info() << "Add file output node:"; 
            // Magically fudge the id values, for now
            ai.input = "image_in";
            no.node = 1;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = AddNodeMessage(nt_file_output, arcs_in, arcs_out);
            info() << "\t" << an;
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            info() << "\tsent" << sent << "bytes";
            
            
            // Set input image parameter
            info() << "Setting input image parameter:";
            SetNodeParameterMessage sp;
            sp.nodeId(1);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("test.jpg");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            
            info() << "Trying to set invalid parameter:";
            sp.paramId("void param");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";

            
            info() << "Setting output image parameter:";
            sp.nodeId(2);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("pt.out0.jpg");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            
            //throw(std::runtime_error("test sequence complete"));
        }
};

static ImagePipelineTesterNode* node;

void cleanup()
{
    info() << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done.";
}

void interrupt(int sig)
{
    info() << std::endl;
    info() << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char **argv)
{
    signal(SIGINT, interrupt);
    node = new ImagePipelineTesterNode("cauv");
    node->run();
    cleanup();
}

