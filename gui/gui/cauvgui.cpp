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

#include "cauvgui.h"

#include <QDir>
#include <QString>
#include <QPluginLoader>
#include <QLibrary>
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

int CauvGui::findPlugins(const QDir& dir, int subdirs)
{
    debug(1) << "Looking for plugins in:"<< dir.absolutePath().toStdString();
    
    int numFound = 0;
    foreach (QString fileName, dir.entryList(QDir::Files)) {
        if (!QLibrary::isLibrary(fileName)) {
            continue;
        }
        QPluginLoader loader(dir.absoluteFilePath(fileName));

        debug(1) << "Trying to load:"<< fileName.toStdString();
        if (!loader.load()) {
            debug(1) << "Could mot load plugin" << fileName.toStdString() << ":" << loader.errorString().toStdString();
        } else {
            QObject *plugin = loader.instance();
            if (!plugin) {
                debug(1) << "Could mot instantiate plugin" << fileName.toStdString() << ":" << loader.errorString().toStdString();
            } else {
                if (loadPlugin(plugin)) {
                    info() << "Loaded plugin"<< fileName.toStdString();
                    numFound++;
                } else warning() << "Rejected plugin"<< fileName.toStdString();
            }
        }
    }
    
    if (subdirs > 0 || subdirs == -1) {
        // Recurse down the directories
        foreach (QString directoryName, dir.entryList(QDir::Dirs | QDir::NoDotAndDotDot)) {
            QDir subdir(dir);
            if(subdir.cd(directoryName)) {
                findPlugins(subdir, subdirs == -1 ? -1 : subdirs - 1);
            }
        }
    }

    return numFound;
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
    findPlugins(pluginsDir, 1);


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
        QStringList groups = basicPlugin->getGroups();
        foreach (QString group, groups){
            joinGroup(group.toStdString());
        }

        // tabs
        QList<QWidget* > widgets = basicPlugin->getCentralWidgets();
        foreach (QWidget * const widget, widgets){
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
