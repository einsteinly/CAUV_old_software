#include <QApplication>

#include <iostream>
#include <sstream>
#include <stdint.h>

#include <QTimer>

#include "cauvgui.h"
#include "gamepad/playstationinput.h"

using namespace std;

static CauvGui* node;

void cleanup()
{
    info() << "Cleaning up..." << endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << endl;
}

void interrupt(int sig)
{
    cout << endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    node = new CauvGui(app);

    try {
        info() << GamepadInput::listDevices();
        PlaystationInput* gi;
        gi = new PlaystationInput(0);

        gi->setParent(node);
        gi->connect(gi, SIGNAL(X(bool)), gi, SLOT(printIt(bool)));
        gi->connect(gi, SIGNAL(Joy_L_X(int)), gi, SLOT(printIt(int)));

        // timer to read the game controller
        QTimer *timer = new QTimer(node);
        timer->connect(timer, SIGNAL(timeout()), gi, SLOT(processEvents()));
        timer->start(200);
    } catch (char const* ex){
        error() << ex;
    }

    signal(SIGINT, interrupt);

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    try {
        node->run();
    } catch (char const* ex){
        error() << ex;
    }
    cleanup();
    return 0;
}

