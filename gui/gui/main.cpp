/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
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

#include <gui/core/mainwindow.h>
#include <gui/core/styles/style.h>

using namespace std;
using namespace cauv;


struct Application : public QApplication{

    Application(int &argc, char **argv):
        QApplication(argc, argv){

    }

    bool notify(QObject * rec, QEvent * ev)
    {
      try
      {
        return QApplication::notify(rec,ev);
      }
      catch(std::exception & e)
      {
            error() << "Exception from signal / slot:" << e.what();
            exit(1);
      }
      return false;
    }
};


int main(int argc, char** argv)
{
    Application app(argc, argv);
    Q_INIT_RESOURCE(resources);
    QTextCodec::setCodecForCStrings(QTextCodec::codecForName("UTF-8"));

    Application::setStyle(new cauv::gui::CauvStyle());

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
        node->run(false);

        info() << "Waiting for CauvNode to finish...";
        while(node->isRunning()) sleep(10);
        node.reset();
        info() << "Finished. Bye";
    } catch (char const* ex){
        error() << ex;
    }

    return 0;
}

