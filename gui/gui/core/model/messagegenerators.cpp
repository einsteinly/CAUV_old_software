#include "messagegenerators.h"

#include <gui/core/model/model.h>

using namespace cauv;
using namespace cauv::gui;


MessageGenerator::MessageGenerator(boost::shared_ptr<AUV> auv) :
        m_auv(auv) {
}
