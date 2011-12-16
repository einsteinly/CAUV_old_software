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


#include <iostream>
#include <sstream>
#include <stdint.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include <QApplication>
#include <QTextCodec>
#include <QFile>

#include <gui/core/framework/mainwindow.h>
#include <gui/core/framework/style.h>

#include <common/cauv_utils.h>

using namespace std;
using namespace cauv;



int main(int argc, char** argv)
{
    QApplication app(argc, argv);
    Q_INIT_RESOURCE(resources);
    QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));

    QApplication::setStyle(new cauv::gui::CauvStyle());

    QFile qss(":/resources/stylesheet.qss");
    qss.open(QFile::ReadOnly);
    info() << QString(qss.readAll()).toStdString();
    app.setStyleSheet(qss.readAll());
    qss.close();

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

