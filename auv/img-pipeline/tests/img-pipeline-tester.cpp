/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include <string>
#include <iostream>
#include <exception>
#include <stdexcept>

#include <boost/make_shared.hpp>
#include <boost/thread.hpp>

#include <common/cauv_node.h>
#include <common/mailbox.h>
#include <generated/messages.h>
#include <debug/cauv_debug.h>

using namespace cauv;

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

        void onNodeAddedMessage(boost::shared_ptr<const NodeAddedMessage> m){
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
            ParamValue pv(ParamType::e(0),0,0,"");

            sp = boost::make_shared<SetNodeParameterMessage>();
            
            // clear the pipeline
            cp = boost::make_shared<ClearPipelineMessage>();
            sent = mailbox()->sendMessage(cp, RELIABLE_MSG);

            // Add input node
            an = boost::make_shared<AddNodeMessage>(NodeType::NetInput, arcs_in, arcs_out); 
            info() << "adding net input node";
            sent = mailbox()->sendMessage(an, RELIABLE_MSG); 
            int input_node_id = m_obs->waitOnNodeAdded();

            // Add output node
            info() << "adding file output node"; 
            ai.input = "image_in";
            no.node = input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(NodeType::FileOutput, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG); 
            int output_node_id = m_obs->waitOnNodeAdded();


            info() << "setting output image";
            sp->nodeId(output_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "file-out.jpg";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);

            info() << "Setting source camera:";
            sp->nodeId(input_node_id);
            sp->paramId("device id");
            pv.type = ParamType::Int32;
            pv.intValue = 0;
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);
        }
            
        void clearPipeline(){
            info() << BashColour::Green << "--- clearing the pipeline ---";
            boost::shared_ptr<ClearPipelineMessage> cp;
            
            // clear the pipeline
            cp = boost::make_shared<ClearPipelineMessage>();
            mailbox()->sendMessage(cp, RELIABLE_MSG);
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
            ParamValue pv(ParamType::e(0),0,0,"");

            sp = boost::make_shared<SetNodeParameterMessage>();
            
            clearPipeline();

            // Add input node
            info() << "Add file input node:";
            an = boost::make_shared<AddNodeMessage>(NodeType::FileInput, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int file_input_node_id = m_obs->waitOnNodeAdded();
            
            // Add output node
            info() << "Add file output node:"; 
            ai.input = "image_in";
            no.node = file_input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(NodeType::FileOutput, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG); 
            int file_output_node_id = m_obs->waitOnNodeAdded();
            
            // Set input image parameter
            info() << "Setting input image parameter:"; 
            sp->nodeId(file_input_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "test.jpg";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);
            
            info() << "Trying to set invalid parameter:";
            sp->paramId("void param");
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG); 
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "pt-out0.png";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG); 
            
            info() << "Setting output image compression:";
            sp->nodeId(file_output_node_id);
            sp->paramId("jpeg quality");
            pv.type = ParamType::Int32;
            pv.intValue = 80;
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG); 
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "pt-out1.tiff";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "pt-out0.png";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);
            
            info() << "Setting output image parameter:";
            sp->nodeId(file_output_node_id); 
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "pt-out0.jpg";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);
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
            ParamValue pv(ParamType::e(0),0,0,"");

            sp = boost::make_shared<SetNodeParameterMessage>();

            clearPipeline();

            // Add input node
            an = boost::make_shared<AddNodeMessage>(NodeType::CameraInput, arcs_in, arcs_out); 
            info() << "adding camera input node";
            sent = mailbox()->sendMessage(an, RELIABLE_MSG); 
            int input_node_id = m_obs->waitOnNodeAdded();

            info() << "Setting source camera:";
            sp->nodeId(input_node_id);
            sp->paramId("device id");
            pv.type = ParamType::Int32;
            pv.intValue = 0;
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);

            // Add convert node: default conversion is RGB->Grey
            info() << "Adding Colour Conversion node:";
            ai.input = "image_in";
            no.node = input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::ConvertColour, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int convert_node_id = m_obs->waitOnNodeAdded();
            
            // add Canny node
            info() << "Adding Canny node:";
            ai.input = "image_in";
            no.node = convert_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::Canny, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int canny_node_id = m_obs->waitOnNodeAdded();

            // add Hough lines node
            info() << "Adding Hough lines node:";
            ai.input = "image_in";
            no.node = canny_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::HoughLines, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int hough_node_id = m_obs->waitOnNodeAdded();

            // Add a display node:
            info() << "Adding local display node:";
            ai.input = "image_in";
            no.node = hough_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(NodeType::LocalDisplay, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
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
            ParamValue pv(ParamType::e(0),0,0,"");

            sp = boost::make_shared<SetNodeParameterMessage>();

            clearPipeline();

            // Add input node
            info() << "Add file input node:";
            an = boost::make_shared<AddNodeMessage>(NodeType::FileInput, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int file_input_node_id = m_obs->waitOnNodeAdded();
            
            // Set input image parameter
            info() << "Setting input image filename"; 
            sp->nodeId(file_input_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "test.jpg";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG);

            // Add convert node: default conversion is RGB->Grey
            info() << "Adding Colour Conversion node:";
            ai.input = "image_in";
            no.node = file_input_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::ConvertColour, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int convert_node_id = m_obs->waitOnNodeAdded();
            
            info() << "Setting number of channels for conversion output";
            sp->nodeId(convert_node_id);
            sp->paramId("channels");
            pv.type = ParamType::Int32;
            pv.intValue = 1;
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG); 
            
            // add Canny node
            info() << "Adding Canny node:";
            ai.input = "image_in";
            no.node = convert_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::Canny, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int canny_node_id = m_obs->waitOnNodeAdded();

            // add Hough lines node
            info() << "Adding Hough lines node:";
            ai.input = "image_in";
            no.node = canny_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai); 
            an = boost::make_shared<AddNodeMessage>(NodeType::HoughLines, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG);
            int hough_node_id = m_obs->waitOnNodeAdded();

            // Add output node
            info() << "Add file output node:"; 
            ai.input = "image_in";
            no.node = hough_node_id;
            no.output = "image_out";
            ai.src = no;
            arcs_in.clear();
            arcs_in.push_back(ai);
            an = boost::make_shared<AddNodeMessage>(NodeType::FileOutput, arcs_in, arcs_out);
            sent = mailbox()->sendMessage(an, RELIABLE_MSG); 
            int file_output_node_id = m_obs->waitOnNodeAdded();
            
            info() << "Setting output image filename:";
            sp->nodeId(file_output_node_id);
            sp->paramId("filename");
            pv.type = ParamType::String;
            pv.stringValue = "hough_out.jpg";
            sp->value(pv);
            sent = mailbox()->sendMessage(sp, RELIABLE_MSG); 
        }
        
        virtual void onRun(){
            info() << "--- ImagePipelineTesterNode::onRun ---"; 
            #if defined(USE_DEBUG_MESSAGE_OBSERVERS) 
            info() << "Add TestMBObserver...";
            addMessageObserver(boost::shared_ptr<TestMBObserver>(new TestMBObserver)); 
            #endif
            
            info() << "Joining pl_gui group...";
            joinGroup("pl_gui"); 

            info() << "--- start test sequence ---";

            if(!m_tests_to_run)
                info() << "--- no tests to run ---";

            m_obs = boost::make_shared<NodeAddedObserver>();
            addMessageObserver(m_obs);
    
            if(m_tests_to_run & file_io_test){
                setupFileIOTests();
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            }

            if(m_tests_to_run & fileinput_test){
                setupFileInputToFile();
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            }
            
            if(m_tests_to_run & camera_test){
                setupCameraToDisplay();
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            }
            
            if(m_tests_to_run & hough_test){
                setupHoughTest();
                boost::this_thread::sleep(boost::posix_time::milliseconds(2000));
            }

            raise(SIGTERM);
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
    int tests = 0;
    for(int arg = 1; arg < argc; arg++){
        std::string s = argv[arg];
        if(s == "file_io_test") tests |= file_io_test;
        else if(s == "fileinput_test") tests |= fileinput_test;
        else if(s == "camera_test") tests |= camera_test;
        else if(s == "hough_test") tests |= hough_test;
        else{
            std::cerr << "Unrecognised argument: " << s << std::endl;
            std::cerr << "Usage:\n"
                      << argv[0]
                      << " [fileinput_test]"
                      << " [file_io_test]"
                      << " [camera_test]"
                      << " [hough_test]" << std::endl;
            return 1;
        }
    }

    node = new ImgPipeTestNode(tests);
    node->run(false);
    cleanup();
    return 0;
}


