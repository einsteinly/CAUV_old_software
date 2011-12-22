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

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDir>
#include <QString>
#include <QPluginLoader>
#include <QSettings>
#include <QGLWidget>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <debug/cauv_debug.h>

#include "cauvplugins.h"

#include "model/model.h"
#include "model/registry.h"
#include "model/messageobserver.h"

#include "framework/nodescene.h"
#include "framework/nodepicker.h"

#include "fluidity/view.h"

//!!! just for testing
#include <gui/core/model/nodes/numericnode.h>
#include <common/bounded_float.h>


using namespace cauv;
using namespace cauv::gui;

CauvMainWindow::CauvMainWindow(QApplication * app) :
        CauvNode("CauvGui"),
        m_application(app),
        m_actions(boost::make_shared<GuiActions>()),
        ui(new Ui::MainWindow) {

    ui->setupUi(this);

    ui->streamsDock->setTitleBarWidget(new QWidget());

    // more misc ui setup
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    this->setWindowState(Qt::WindowMaximized);
}

CauvMainWindow::~CauvMainWindow(){
    delete ui;
}


void CauvMainWindow::closeEvent(QCloseEvent* e){
    QSettings settings("CAUV", "Cambridge AUV");
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());

    hide();
    m_application->quit();

    QMainWindow::closeEvent(e);
}

int CauvMainWindow::send(boost::shared_ptr<const Message> message){
    debug(0) << "Sending message: " << *(message.get());
    return CauvNode::send(message);
}

void CauvMainWindow::onRun()
{
    CauvNode::onRun();

    // data model and network access
    // !!! todo: there shouldn't be a RedHerring class
    // vehicle data should be sent from the AUV to a
    // more generic version of the Vehicle class. Things
    // like motor setup, autopilots, etc.. should be
    // defined there not hardcoded into the GUI.
    m_actions->auv = VehicleRegistry::instance()->registerVehicle<RedHerring>("redherring");
    m_actions->root = boost::make_shared<NodeItemModel>(VehicleRegistry::instance());

    // cauv node
    m_actions->node = shared_from_this();
    
    // Exposed interface elements - plugins might need to access some
    // elements of the main GUI framework. Here's where we can pass
    // these bits in.
    // NodePicker is exposed
    m_actions->nodes = new NodePicker(m_actions->root);
    ui->streamsDock->setWidget(m_actions->nodes);
    // And the main window
    m_actions->window = shared_from_this();
    // The main view onto the scene
    this->setCentralWidget(m_actions->view = new liquid::LiquidView());
    // And ofcourse the scene itself
    m_actions->scene = boost::make_shared<NodeScene>();
    m_actions->view->setScene(m_actions->scene.get());
    m_actions->view->centerOn(0,0);

    // Set the viewport to use OpenGl here. Nested Gl viewports don't work
    m_actions->view->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));


    // message input
    this->addMessageObserver(boost::make_shared<DefaultGuiMessageObserver>(m_actions->auv));
    this->addMessageObserver(boost::make_shared<DebugMessageObserver>(5));

    // always need at least the gui and control group
    this->joinGroup("gui");
    this->joinGroup("control");

    // forward messages from "set" events
    connect(m_actions->auv.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
            this, SLOT(send(boost::shared_ptr<const Message>)));

    // Load external plugins (this includes things like gamepad support)
    // static plugins first
    foreach (QObject *plugin, QPluginLoader::staticInstances())
        loadPlugin(plugin);

    // then any plugins in the plugins folder
    QDir pluginsDir = QDir(QApplication::instance()->applicationDirPath());
    debug() << "Loading dynamic plugins from" << pluginsDir.absolutePath().toLatin1().constData();
    pluginsDir.cd("plugins");
    findPlugins(pluginsDir, 1);


    QSettings settings("CAUV", "Cambridge AUV");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());


    //!!! just for testing
    boost::shared_ptr<NumericNode<BoundedFloat> > n = m_actions->auv->findOrCreate<GroupingNode>("test")->findOrCreate<NumericNode<BoundedFloat> >(MotorID::Prop);
    n->setMutable(true);
    n->update(QVariant::fromValue(BoundedFloat(3.0, 0, 100, BoundedFloatType::Clamps)));
    n->setNeutral(0);

    show();
    m_application->exec();

    foreach(CauvInterfacePlugin * plugin, m_plugins){
        plugin->shutdown();
        delete plugin;
    }

    info() << "Qt Thread exiting";
    info() << "Stopping CauvNode";
    CauvNode::stopNode();
}

int CauvMainWindow::findPlugins(const QDir& dir, int subdirs)
{
    debug(3) << "Looking for plugins in:"<< dir.absolutePath().toStdString();

    int numFound = 0;
    foreach (QString fileName, dir.entryList(QDir::Files)) {
        QPluginLoader loader(dir.absoluteFilePath(fileName));
        QObject *plugin = loader.instance();
        if (plugin) {
            if (CauvInterfacePlugin * cauvPlugin= loadPlugin(plugin)) {
                m_plugins.push_back(cauvPlugin);
                info() << "Loaded plugin:"<< fileName.toStdString();
                numFound++;
            } else {
                plugin->deleteLater();
                warning() << "Rejected plugin:"<< fileName.toStdString();
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


CauvInterfacePlugin * CauvMainWindow::loadPlugin(QObject *plugin){

    // all plugins must come from CauvInterfacePlugin
    CauvInterfacePlugin * basicPlugin = qobject_cast<CauvInterfacePlugin*>(plugin);
    if(basicPlugin) {
        basicPlugin->initialise(m_actions);
        return basicPlugin;
    }

    return NULL;
}
