#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <common/cauv_utils.h>
#include <generated/messages.h>
#include <common/spread/spread_mailbox.h>

using namespace cauv;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Error: Message type required!" << std::endl;
        std::cerr << "USAGE: send MESSAGETYPE [PARAMS]" << std::endl;
        return 1;
    }

    std::string msgType(argv[1]);

    SpreadMailbox m_mailbox;
    m_mailbox.connect("16707@localhost", "send");
    
    if (boost::iequals(msgType, "resetmcb")) {
        if (argc - 2 != 0) {
            std::cerr << "Error: reset MCB message requires exactly zero parameters" << std::endl;
            return 3;
        }
        
        boost::shared_ptr<ResetMCBMessage> m = boost::make_shared<ResetMCBMessage>();
    
        std::cout << "Sending reset MCB message " << *m << std::endl;
        m_mailbox.sendMessage(m, SAFE_MESS);
    }
    else if (boost::iequals(msgType, "motor")) {
        if (argc - 2 != 2) {
            std::cerr << "Error: motor message requires exactly two parameters (motorid, speed)" << std::endl;
            return 3;
        }
        uint8_t motorid = boost::lexical_cast<unsigned int>(argv[2]);
        int8_t speed = boost::lexical_cast<int>(argv[3]);

        boost::shared_ptr<MotorMessage> m = boost::make_shared<MotorMessage>((MotorID::e)motorid, speed);

        std::cout << "Sending motor message " << *m << std::endl;
        m_mailbox.sendMessage(m, SAFE_MESS);
    }
    else if (boost::iequals(msgType, "trace")) {
        std::string message = "Hiyoooooooooooo!";

        boost::shared_ptr<DebugMessage> m = boost::make_shared<DebugMessage>(DebugType::Trace, message);

        std::cout << "Sending trace message " << *m << std::endl;
        m_mailbox.sendMessage(m, SAFE_MESS);
    }
    else if (boost::iequals(msgType, "autopilot")) {
        if (argc - 2 != 3) {
            std::cerr << "Error: autopilot message requires exactly three parameters (autopilot, enabled, target)" << std::endl;
            return 3;
        }
        int autopilot = boost::lexical_cast<unsigned int>(argv[2]);
        bool enabled = boost::lexical_cast<bool>(argv[3]);
        float target = boost::lexical_cast<float>(argv[4]);

        switch(autopilot){
        case 0: {
            boost::shared_ptr<BearingAutopilotEnabledMessage> m = boost::make_shared<BearingAutopilotEnabledMessage>(enabled, target);
            std::cout << "Sending autopilot message " << *m << std::endl;
            m_mailbox.sendMessage(m, SAFE_MESS);
        }
        break;

        case 1: {
            boost::shared_ptr<PitchAutopilotEnabledMessage> m = boost::make_shared<PitchAutopilotEnabledMessage>(enabled, target);
            std::cout << "Sending autopilot message " << *m << std::endl;
            m_mailbox.sendMessage(m, SAFE_MESS);
        }
        break;

        case 2: {
            boost::shared_ptr<DepthAutopilotEnabledMessage> m = boost::make_shared<DepthAutopilotEnabledMessage>(enabled, target);
            std::cout << "Sending autopilot message " << *m << std::endl;
            m_mailbox.sendMessage(m, SAFE_MESS);
        }
        break;
        default:
            std::cerr << "Error: Unknown autopilot specified." << std::endl;
            return 3;
        }
    }
    else if (boost::iequals(msgType, "telemetryspam")) {
        if (argc - 2 != 4) {
            std::cerr << "Error: telemetry message requires exactly four parameters (orientation.ypr, depth)" << std::endl;
            return 3;
        }


        while(true)
        {
            floatYPR orientation;
            orientation.yaw = rand();//boost::lexical_cast<float>(argv[2]);
            orientation.pitch = rand();//boost::lexical_cast<float>(argv[3]);
            orientation.roll = rand();//boost::lexical_cast<float>(argv[4]);
            float depth = rand();//boost::lexical_cast<float>(argv[5]);
            boost::shared_ptr<TelemetryMessage> m = boost::make_shared<TelemetryMessage>(orientation, depth);

            usleep(100000);

            std::cout << "Sending telemetry message " << *m << std::endl;
            m_mailbox.sendMessage(m, SAFE_MESS);
        }
    }
    else {
        std::cerr << "Error: unknown message type " << msgType << std::endl;
        return 2;
    }
    
    
    return 0;
}
