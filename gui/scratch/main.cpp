/* Copyright 2011 Cambridge Hydronautics Ltd.
 *
 * Cambridge Hydronautics Ltd. licenses this software to the CAUV student
 * society for all purposes other than publication of this source code.
 *
 * See license.txt for details.
 *
 * Please direct queries to the officers of Cambridge Hydronautics:
 *     James Crosby    james@camhydro.co.uk
 *     Andy Pritchard   andy@camhydro.co.uk
 *     Leszek Swirski leszek@camhydro.co.uk
 *     Hugo Vincent     hugo@camhydro.co.uk
 */

#include <QtGui>
#include <QThread>

#include <boost/make_shared.hpp>

#include <common/cauv_node.h>

#include "style.h"
#include "fluidity/view.h"

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

    QIcon icon;
    icon.addFile(QString::fromUtf8(":/resources/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
    app.setWindowIcon(icon);

    QApplication::setStyle(new cauv::gui::CauvStyle());

    boost::shared_ptr<ScratchNode> node = boost::make_shared<ScratchNode>("scratch");
    node->parseOptions(argc, argv);
    node->start();

    cauv::gui::f::FView view(node);
    view.show();

    int exit_status = app.exec();
    
    node->stopNode();

    return exit_status;
}
