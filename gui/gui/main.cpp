#include <QApplication>
#include <QPushButton>

#include <iostream>
#include <sstream>
#include <stdint.h>

#include "cauvgui.h"

using namespace std;

static CauvGui* node;
static QApplication* app;

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
    app = new QApplication(argc, argv);

    signal(SIGINT, interrupt);

    node = new CauvGui();

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    node->run();

    cleanup();
    return 0;
}

