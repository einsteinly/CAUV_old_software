#include "cauvgui.h"

#include <QDir>
#include <QString>
#include <QPluginLoader>
#include <QSettings>

#include <model/auv_controller.h>
#include <model/auv_model.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <gui/core/cauvplugins.h>

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "ui_mainwindow.h"

using namespace cauv;

CauvGui::CauvGui(QApplication * app) :
        CauvNode("CauvGui"),
        m_auv( boost::make_shared<AUV>()),
        m_auv_controller(boost::make_shared<AUVController>(m_auv)),
        m_application(app),
        ui(new Ui::MainWindow) {

    ui->setupUi(this);

    // more misc ui setup
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    this->setWindowState(Qt::WindowMaximized);
}

CauvGui::~CauvGui(){
    delete ui;
}

void CauvGui::addCentralTab(QWidget* const tab, QString& name){
    addCentralTab(tab, (const QString&)name);
}

void CauvGui::addCentralTab(QWidget* const tab, const QString& name){
    info() << "Registering central screen [" << name.toStdString() << "]";
    ui->tabWidget->addTab(tab, name);
}

void CauvGui::addDock(QDockWidget* dock, Qt::DockWidgetArea area){
    info() << "Registering dock widget";
    addDockWidget(area, dock);
}

void CauvGui::closeEvent(QCloseEvent* e){
    QSettings settings("CAUV", "Cambridge AUV");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

    hide();
    m_application->quit();

    QMainWindow::closeEvent(e);
}

int CauvGui::send(boost::shared_ptr<Message> message){
    debug(5) << "Sending message: " << *message;
    return CauvNode::send(message);
}

void CauvGui::onRun()
{
    CauvNode::onRun();

    // load plugins
    // static plugins first
    foreach (QObject *plugin, QPluginLoader::staticInstances())
        loadPlugin(plugin);

    // then any plugins in the plugins folder
    QDir pluginsDir = QDir(QApplication::instance()->applicationDirPath());
    pluginsDir.cd("plugins");

    foreach (QString directoryName, pluginsDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {


        if(pluginsDir.cd(directoryName)){

            debug(3) << "Looking for plugins in:"<< pluginsDir.absolutePath().toStdString();

            foreach (QString fileName, pluginsDir.entryList(QDir::Files)) {
                QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
                QObject *plugin = loader.instance();
                if (plugin) {
                    if (loadPlugin(plugin)) {
                        info() << "Loaded plugin:"<< fileName.toStdString();
                    } else warning() << "Rejected plugin:"<< fileName.toStdString();
                }
            }

            pluginsDir.cdUp(); // back to plugins dir
        }
    }


    QSettings settings("CAUV", "Cambridge AUV");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());


    // connect up message inputs and outputs
    addMessageObserver(m_auv_controller);
    m_auv_controller->onMessageGenerated.connect(boost::bind(&CauvGui::send, this, _1));

    show();
    m_application->exec();

    info() << "Qt Thread exiting";
    removeMessageObserver(m_auv_controller);
    info() << "Stopping CauvNode";
    CauvNode::stopNode();
}


bool CauvGui::loadPlugin(QObject *plugin){

    // all plugins must come from CauvInterfacePlugin
    CauvInterfacePlugin *basicPlugin = qobject_cast<CauvInterfacePlugin*>(plugin);
    if(basicPlugin) {
        basicPlugin->initialise(m_auv, shared_from_this());
        // see which groups the plugin needs us to join
        foreach (QString group, basicPlugin->getGroups()){
            joinGroup(group.toStdString());
        }

        // tabs
        foreach (QWidget * const widget, basicPlugin->getCentralWidgets()){
            addCentralTab(widget, basicPlugin->name());
        }
        // docks
        QMapIterator<QDockWidget*, Qt::DockWidgetArea> i(basicPlugin->getDockWidgets());
        while (i.hasNext()) {
            i.next();
            addDockWidget(i.value(), i.key());
        }
    }

    return (basicPlugin ? true : false);
}
