#include <iostream>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/make_shared.hpp>

#include <common/messages.h>
#include <common/spread/spread_mailbox.h>

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Error: Message type required!" << std::endl;
        std::cerr << "USAGE: send MESSAGETYPE [PARAMS]" << std::endl;
        return 1;
    }

    std::string msgType(argv[1]);

    SpreadMailbox m_mailbox("16707@localhost", "send");
    
    if (boost::iequals(msgType, "motor")) {
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
    else {
        std::cerr << "Error: unknown message type " << msgType << std::endl;
        return 2;
    }
    
    
    return 0;
}
