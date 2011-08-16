
#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <QApplication>
#include <QTextCodec>

#include <gui/core/framework/mainwindow.h>

#include <common/cauv_utils.h>

using namespace std;
using namespace cauv;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    Q_INIT_RESOURCE(resources);
    QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));

    QIcon icon;
    icon.addFile(QString::fromUtf8(":/resources/icon.png"), QSize(), QIcon::Normal, QIcon::Off);
    app.setWindowIcon(icon);

    boost::shared_ptr<gui::CauvMainWindow> node = boost::make_shared<gui::CauvMainWindow>(&app);

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    try {
        node->run();

        info() << "Waiting for CauvNode to finish...";
        while(node->isRunning()) sleep(10);
        node.reset();
        info() << "Finished. Bye";
    } catch (char const* ex){
        error() << ex;
    }

    return 0;
}

