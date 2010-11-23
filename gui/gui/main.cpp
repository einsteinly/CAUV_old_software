#include <QApplication>
#include <QPushButton>

#include <iostream>
#include <sstream>

#include <common/cauv_node.h>
#include <debug/cauv_debug.h>

#include "ui_mainwindow.h"


class CauvGui : public QMainWindow, public CauvNode, private Ui::MainWindow {
    public:
        CauvGui(QWidget *parent = 0) : CauvNode("CauvGui"){
            setupUi(this);
        }

        void onRun()
        {

            CauvNode::onRun();
            show();
        }
};


static CauvGui* node;

void cleanup()
{
    info() << "Cleaning up..." << std::endl;
    CauvNode* oldnode = node;
    node = 0;
    delete oldnode;
    info() << "Clean up done." << std::endl;
}

void interrupt(int sig)
{
    std::cout << std::endl;
    info() << BashColour::Red << "Interrupt caught!";
    cleanup();
    signal(SIGINT, SIG_DFL);
    raise(sig);
}

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    signal(SIGINT, interrupt);
    node = new CauvGui();
    node->run();
    app.exec();
    cleanup();
    return 0;
}

