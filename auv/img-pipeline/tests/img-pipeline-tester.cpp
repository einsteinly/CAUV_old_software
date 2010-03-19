#include <string>
#include <iostream>
#include <exception>
#include <stdexcept>

#include <boost/make_shared.hpp>

#include <common/cauv_node.h>
#include <common/messages.h>
#include <common/debug.h>

#include <common/spread/cauv_mailbox_monitor.h>


enum test_e{
    file_io_test = 1,
    fileinput_test = 2,
    camera_test = 4,
    hough_test = 8
};

class NodeAddedObserver: public MessageObserver{
    public:
        NodeAddedObserver()
            : MessageObserver(), m_node_id(0){
        }

        void onNodeAddedMessage(boost::shared_ptr<NodeAddedMessage> m){
            boost::lock_guard<boost::recursive_mutex> l(m_node_id_mutex);
            m_node_id = m->nodeId();
        }

        int waitOnNodeAdded(){
            int orig_id = m_node_id;
            while(getLastID() == orig_id){
                boost::this_thread::sleep(boost::posix_time::milliseconds(100));
            }
            return getLastID();
        }

        int getLastID(){
            boost::lock_guard<boost::recursive_mutex> l(m_node_id_mutex);
            return m_node_id;
        }

    private:
        boost::recursive_mutex m_node_id_mutex;
        int m_node_id; 
};

class ImgPipeTestNode : public CauvNode{
    public:
        ImgPipeTestNode(int tests_to_run)
            : CauvNode("pipe-test"), m_tests_to_run(tests_to_run){
        }

    protected:
        void setupFileInputToFile(){
            info() << BashColour::Green << "--- Setting up pipeline for fileinput->file ---";
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;
            boost::shared_ptr<AddNodeMessage> an;
            boost::shared_ptr<SetNodeParameterMessage> sp;
            boost::shared_ptr<ClearPipelineMessage> cp;
            
            // clear the pipeline
            cp = boost::make_shared<ClearPipelineMessage>();
            sent = mailbox()->sendMessage(cp, SAFE_MESS);

            // Add input node
            an = boost::make_shared<AddNodeMessage>(nt_net_input, arcs_in, arcs_out); 
            info() << "adding net input node";
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            int input_node_id = m_obs->waitOnNodeAdded();

            // Add output node
            info() << "adding file output node"; 
            ai.input = "image_in";
            no.node = input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(nt_file_output, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            int output_node_id = m_obs->waitOnNodeAdded();


            info() << "setting output image";
            // initialise everything to supress valgrind's complaints
            sp = boost::make_shared<SetNodeParameterMessage>(0, "", pt_int32, 0, 0, "");
            sp->nodeId(output_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("file-out.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);

            info() << "Setting source camera:";
            sp->nodeId(input_node_id);
            sp->paramId("camera id");
            sp->stringValue("");
            sp->paramType(pt_int32);
            sp->intValue(cam_forward);
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
        }
            
        void clearPipeline(){
            info() << BashColour::Green << "--- clearing the pipeline ---";
            boost::shared_ptr<ClearPipelineMessage> cp;
            
            // clear the pipeline
            cp = boost::make_shared<ClearPipelineMessage>();
            mailbox()->sendMessage(cp, SAFE_MESS);
        }

        void setupFileIOTests(){
            info() << BashColour::Green << "--- Setting up pipeline for file->file tests ---";
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;
            boost::shared_ptr<AddNodeMessage> an;
            boost::shared_ptr<SetNodeParameterMessage> sp;
            
            clearPipeline();

            // Add input node
            info() << "Add file input node:";
            an = boost::make_shared<AddNodeMessage>(nt_file_input, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int file_input_node_id = m_obs->waitOnNodeAdded();
            
            // Add output node
            info() << "Add file output node:"; 
            ai.input = "image_in";
            no.node = file_input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(nt_file_output, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            int file_output_node_id = m_obs->waitOnNodeAdded();
            
            // Set input image parameter
            info() << "Setting input image parameter:";
            // initialise everything to supress valgrind's complaints 
            sp = boost::make_shared<SetNodeParameterMessage>(0, "", pt_int32, 0, 0, ""); 
            sp->nodeId(file_input_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("img-pipeline/tests/test.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            
            info() << "Trying to set invalid parameter:";
            sp->paramId("void param");
            sent = mailbox()->sendMessage(sp, SAFE_MESS); 
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("pt-out0.png");
            sent = mailbox()->sendMessage(sp, SAFE_MESS); 
            
            info() << "Setting output image compression:";
            sp->nodeId(file_output_node_id);
            sp->paramId("jpeg quality");
            sp->paramType(pt_int32);
            sp->intValue(80);
            sent = mailbox()->sendMessage(sp, SAFE_MESS); 
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("pt-out1.tiff");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("pt-out0.png");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id); 
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("pt-out0.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);
        }

        void setupCameraToDisplay(){
            info() << BashColour::Green << "--- Setting up pipeline for camera->display ---";
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;
            boost::shared_ptr<AddNodeMessage> an;
            boost::shared_ptr<SetNodeParameterMessage> sp;

            clearPipeline();

            // Add input node
            an = boost::make_shared<AddNodeMessage>(nt_camera_input, arcs_in, arcs_out); 
            info() << "adding camera input node";
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            int input_node_id = m_obs->waitOnNodeAdded();

            info() << "Setting source camera:";
            sp = boost::make_shared<SetNodeParameterMessage>(0, "", pt_int32, 0, 0, ""); 
            sp->nodeId(input_node_id);
            sp->paramId("camera id");
            sp->stringValue("");
            sp->paramType(pt_int32);
            sp->intValue(cam_forward);
            sent = mailbox()->sendMessage(sp, SAFE_MESS);

            // Add convert node: default conversion is RGB->Grey
            info() << "Adding Colour Conversion node:";
            ai.input = "image_in";
            no.node = input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_convert_colour, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int convert_node_id = m_obs->waitOnNodeAdded();
            
            // add Canny node
            info() << "Adding Canny node:";
            ai.input = "image_in";
            no.node = convert_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_canny, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int canny_node_id = m_obs->waitOnNodeAdded();

            // add Hough lines node
            info() << "Adding Hough lines node:";
            ai.input = "image_in";
            no.node = canny_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_hough_linesp, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int hough_node_id = m_obs->waitOnNodeAdded();

            // Add a display node:
            info() << "Adding local display node:";
            ai.input = "image_in";
            no.node = hough_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(nt_local_display, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            m_obs->waitOnNodeAdded();
        }

        void setupHoughTest(){
            info() << BashColour::Green << "--- Setting up pipeline for Hough test ---";
            std::vector<NodeInputArc> arcs_in;
            std::vector<NodeOutputArc> arcs_out;
            NodeInputArc ai;
            NodeOutputArc ao;
            NodeInput ni;
            NodeOutput no;
            int sent = 0;
            boost::shared_ptr<AddNodeMessage> an;
            boost::shared_ptr<SetNodeParameterMessage> sp;

            clearPipeline();

            // Add input node
            info() << "Add file input node:";
            an = boost::make_shared<AddNodeMessage>(nt_file_input, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int file_input_node_id = m_obs->waitOnNodeAdded();
            
            // Set input image parameter
            info() << "Setting input image filename";
            // initialise everything to supress valgrind's complaints 
            sp = boost::make_shared<SetNodeParameterMessage>(0, "", pt_int32, 0, 0, ""); 
            sp->nodeId(file_input_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("img-pipeline/tests/test.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS);

            // Add convert node: default conversion is RGB->Grey
            info() << "Adding Colour Conversion node:";
            ai.input = "image_in";
            no.node = file_input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_convert_colour, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int convert_node_id = m_obs->waitOnNodeAdded();
            
            info() << "Setting number of channels for conversion output";
            sp->nodeId(convert_node_id);
            sp->paramId("channels");
            sp->paramType(pt_int32);
            sp->intValue(1);
            sent = mailbox()->sendMessage(sp, SAFE_MESS); 
            
            // add Canny node
            info() << "Adding Canny node:";
            ai.input = "image_in";
            no.node = convert_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_canny, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int canny_node_id = m_obs->waitOnNodeAdded();

            // add Hough lines node
            info() << "Adding Hough lines node:";
            ai.input = "image_in";
            no.node = canny_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(nt_hough_linesp, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS);
            int hough_node_id = m_obs->waitOnNodeAdded();

            // Add output node
            info() << "Add file output node:"; 
            ai.input = "image_in";
            no.node = hough_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(nt_file_output, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, SAFE_MESS); 
            int file_output_node_id = m_obs->waitOnNodeAdded();
            
            info() << "Setting output image filename:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            sp->paramType(pt_string);
            sp->stringValue("hough_out.jpg");
            sent = mailbox()->sendMessage(sp, SAFE_MESS); 
        }
        
        virtual void onRun(){
            info() << "--- ImagePipelineTesterNode::onRun ---"; 
            info() << "Add TestMBObserver...";
            eventMonitor()->addObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
            
            info() << "Joining pipeline group...";
            mailbox()->joinGroup("pipeline"); 

            info() << "--- start test sequence ---";

            if(!m_tests_to_run)
                info() << "--- no tests to run ---";

            m_obs = boost::make_shared<NodeAddedObserver>();
            mailboxMonitor()->addObserver(m_obs);
    
            if(m_tests_to_run & file_io_test){
                setupFileIOTests();
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            }

            if(m_tests_to_run & fileinput_test){
                setupFileInputToFile();
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            }
            
            if(m_tests_to_run & camera_test){
                setupCameraToDisplay();
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            }
            
            if(m_tests_to_run & hough_test){
                setupHoughTest();
                boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
            }
        }
    private:
        boost::shared_ptr<NodeAddedObserver> m_obs;
        int m_tests_to_run;
};

static ImgPipeTestNode* node;

void cleanup()
{
    info() << BashColour::Red << "Cleaning up...";
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << BashColour::Red << "Clean up done.";
}

void interrupt(int sig)
{
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char **argv)
{
    signal(SIGINT, interrupt);
    node = new ImgPipeTestNode(hough_test);
    node->run();
    cleanup();
    return 0;
}


