#include <string>
#include <iostream>
#include <exception>
#include <stdexcept>

#include <common/cauv_node.h>
#include <common/messages.h>

#include <common/spread/cauv_mailbox_monitor.h>


class ImagePipelineTesterNode : public CauvNode{
    public:
        ImagePipelineTesterNode(std::string const& group)
            : CauvNode("pipe-test", group){
        }
    protected:
        virtual void onRun(){
            std::cout << "--- ImagePipelineTesterNode::onRun ---" << std::endl;
            
            std::cout << "Add TestMBObserver..." << std::endl;
            eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
            
            std::cout << "Joining pipeline group..." << std::endl;
            mailbox()->joinGroup("pipeline");
            
            std::cout << "--- start test sequence ---" << std::endl;
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
            std::cout << "Add file input node: " << std::flush;
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            std::cout << "sent " << sent << " bytes." << std::endl;

            // Add output node
            std::cout << "Add file output node: " << std::flush; 
            an.nodeType(nt_file_output);
            // Magically fudge the id values, for now
            ai.input = "image_in";
            no.node = 1;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            std::cout << "sent " << sent << " bytes." << std::endl;
            
            //throw(std::runtime_error("test sequence complete"));
        }
};

static ImagePipelineTesterNode* node;

void cleanup()
{
    std::cout << "Cleaning up..." << std::endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    std::cout << "Clean up done." << std::endl;
}

void interrupt(int sig)
{
    std::cout << std::endl;
    std::cout << "Interrupt caught!" << std::endl;
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

