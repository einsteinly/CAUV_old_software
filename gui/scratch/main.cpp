#include <QtGui>
#include <QThread>

#include <boost/make_shared.hpp>

#include <common/cauv_node.h>

#include "viewWidget.h"

class ScratchNode: public cauv::CauvNode, public QThread{
    public:
        ScratchNode(std::string const& n)
            : cauv::CauvNode(n){
        }
        void run(){
            cauv::CauvNode::run();
        }
};

int main(int argc, char *argv[]){
    QApplication app(argc, argv);
    
    boost::shared_ptr<ScratchNode> node = boost::make_shared<ScratchNode>("scratch");
    node->parseOptions(argc, argv);
    node->start();

    cauv::gui::f::ViewWidget view(node);
    view.show();

    int exit_status = app.exec();
    
    node->stopNode();

    return exit_status;
}
