
#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <QTimer>
#include <QApplication>
#include <QTextCodec>

#include "cauvgui.h"

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

    boost::shared_ptr<CauvGui> node = boost::make_shared<CauvGui>(&app);

    int ret = node->parseOptions(argc, argv);
    if(ret != 0) return ret;

    try {
        node->run();

        info() << "Waiting for CauvNode to finish...";
        while(node->isRunning()) sleep(10);
        info() << "Finished. Bye";
    } catch (char const* ex){
        error() << ex;
    }

    return 0;
}

