
#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <QTimer>
#include <QApplication>

#include "cauvgui.h"
#include "gamepad/playstationinput.h"

using namespace std;
using namespace cauv;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    boost::shared_ptr<CauvGui> node = boost::make_shared<CauvGui>(app);

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    try {
        info() << GamepadInput::listDevices();
        PlaystationInput* gi;
        gi = new PlaystationInput(0);

        gi->setParent(node.get());
        gi->connect(gi, SIGNAL(X(bool)), gi, SLOT(printIt(bool)));
        gi->connect(gi, SIGNAL(Joy_L_X(int)), gi, SLOT(printIt(int)));

        // timer to read the game controller
        QTimer *timer = new QTimer(node.get());
        timer->connect(timer, SIGNAL(timeout()), gi, SLOT(processEvents()));
        timer->start(200);
    } catch (char const* ex){
        error() << ex;
    }

    try {
        node->run();
    } catch (char const* ex){
        error() << ex;
    }
    return 0;
}

