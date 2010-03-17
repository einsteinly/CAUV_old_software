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
        ImagePipelineTesterNode()
            : CauvNode("pipe-test"){
        }
    protected:
        void fileIOTests(){
            info() << green << "--- File IO Tests ---";
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
            SetNodeParameterMessage sp(0, "", pt_int32, 0, 0, ""); // initialise everything to supress valgrind's complaints
            sp.nodeId(1);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("img-pipeline/tests/test.jpg");
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
            sp.stringValue("pt-out0.png");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";

            
            info() << "Setting output image compression:";
            sp.nodeId(2);
            sp.paramId("jpeg quality");
            sp.paramType(pt_int32);
            sp.intValue(80);
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            
            
            info() << "Setting output image parameter:";
            sp.nodeId(2);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("pt-out1.tiff");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            
            info() << "Setting output image parameter:";
            sp.nodeId(2);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("pt-out0.png");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
            
            info() << "Setting output image parameter:";
            sp.nodeId(2);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("pt-out0.jpg");
            info() << "\t" << sp;
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            info() << "\tsent" << sent << "bytes";
        }
        
        void cameraInputTests(){
            info() << green << "--- Camera Input Tests ---";
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;

            AddNodeMessage an(nt_camera_input, arcs_in, arcs_out);
            
            // Add input node
            info() << "adding camera input node";
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            
            // Add output node
            info() << "adding file output node"; 
            // Magically fudge the id values, for now
            ai.input = "image_in";
            no.node = 3;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = AddNodeMessage(nt_file_output, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);


            info() << "setting output image";
            SetNodeParameterMessage sp(0, "", pt_int32, 0, 0, ""); // initialise everything to supress valgrind's complaints
            sp.nodeId(4);
            sp.paramId("filename");
            sp.paramType(pt_string);
            sp.stringValue("camera-out.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);

            info() << "Setting source camera:";
            sp.nodeId(3);
            sp.paramId("camera id");
            sp.stringValue("");
            sp.paramType(pt_int32);
            sp.intValue(cam_file);
            sent = mailbox()->sendMessage(sp, SAFE_MESS);

        }
        
        virtual void onRun(){
            info() << "--- ImagePipelineTesterNode::onRun ---";
            
            info() << "Add TestMBObserver...";
            eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
            
            info() << "Joining pipeline group...";
            mailbox()->joinGroup("pipeline");
            
            info() << "--- start test sequence ---";

            fileIOTests();
            
            cameraInputTests();
            
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
    node = new ImagePipelineTesterNode();
    node->run();
    cleanup();
}


