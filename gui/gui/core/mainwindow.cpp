/* Copyright 2011-2013 Cambridge Hydronautics Ltd.
 *
 * See license.txt for details.
 */


#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QDir>
#include <QString>
#include <QPluginLoader>
#include <QSettings>
#include <QGLWidget>
#include <QMetaMethod>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include <debug/cauv_debug.h>

#include <liquid/view.h>

#include "cauvplugins.h"

#include "model/nodeItemModel.h"
#include "model/registry.h"
#include "model/nodes/groupingnode.h"

#include "nodescene.h"
#include "nodepicker.h"
#include "connectednode.h"

#include "drag/graphDropHandler.h"
#include "drag/groupDropHandler.h"

#include "elements/style.h"

#include <liquid/magma/radialMenu.h>
#include <liquid/magma/style.h>

using namespace cauv;
using namespace cauv::gui;

// !!! will move stack widget stuff elsewhere

StackWidget::StackWidget(QWidget* parent)
    : QWidget(parent),
      m_title(nullptr),
      m_stack_widget(nullptr),
      m_stack(),
      m_titleAnimation(nullptr){
    
    m_title = new QLabel(this);
    m_stack_widget = new QStackedWidget(this);
    m_titleAnimation = new QPropertyAnimation(m_title, "geometry");
    m_titleAnimation->setDuration(500);

    auto layout = new QVBoxLayout;
    layout->setSpacing(0);

    layout->addWidget(m_stack_widget);
    layout->addWidget(m_title);

    layout->setContentsMargins(0,0,0,0);
    m_stack_widget->setContentsMargins(0,0,0,0);
    m_title->setContentsMargins(0,5,0,5);
    setContentsMargins(0,0,0,0);

    setLayout(layout);
}

void StackWidget::push(QString name, QWidget* widget){
    m_stack.push(QPair<QString,QWidget*>(name, widget));
    m_stack_widget->addWidget(widget);
    m_stack_widget->setCurrentWidget(widget);
    updateTitle();
}

void StackWidget::pop(){
    QPair<QString,QWidget*> popped = m_stack.pop();
    m_stack_widget->removeWidget(popped.second);
    updateTitle();
}

void StackWidget::updateTitle(){
    QString text;
    
    for(int i = 0; i < m_stack.size(); i++){
        text.append(m_stack[i].first);
        if(i != m_stack.size() - 1)
            text.append(" )) ");
    }

    QRect end = m_title->geometry();
    if(text.isEmpty()) {
        //end.setHeight(0);
        m_title->hide();
    } else {
        //end.setHeight(30);
        m_title->show();
    }
    m_titleAnimation->setEndValue(end);
    //m_titleAnimation->start();

    m_title->setText(text);
}


CauvMainWindow::CauvMainWindow(QApplication * app) :
    CauvNode("CauvGui"),
    m_application(app),
    m_actions(boost::make_shared<GuiActions>()),
    ui(new Ui::MainWindow),
    m_view_stack(nullptr){

    ui->setupUi(this);

    ui->streamsDock->setTitleBarWidget(new QWidget());

    // more misc ui setup
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    this->setWindowState(Qt::WindowMaximized);

    setUnifiedTitleAndToolBarOnMac(true);
}

CauvMainWindow::~CauvMainWindow(){
    delete ui;
}

StackWidget* CauvMainWindow::viewStack(){
    return m_view_stack;
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

    ConnectedNode::setMap(new ConnectedNodeMap());

    // data model and network access
    boost::shared_ptr<VehicleRegistry> registry = VehicleRegistry::instance();
    connect(registry.get(), SIGNAL(observerAttached(boost::shared_ptr<MessageObserver>)),
            this, SLOT(registerObserver(boost::shared_ptr<MessageObserver>)));
    connect(registry.get(), SIGNAL(observerDetached(boost::shared_ptr<MessageObserver>)),
            this, SLOT(unregisterObserver(boost::shared_ptr<MessageObserver>)));
    connect(registry.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
            this, SLOT(send(boost::shared_ptr<const Message>)));
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
    m_actions->view = new liquid::LiquidView();
    // The main view onto the front scene
    m_view_stack = new StackWidget();
    this->setCentralWidget(m_view_stack);

    m_view_stack->push(QString(""), m_actions->view);
    
    // And ofcourse the scene itself
    m_actions->scene = boost::make_shared<NodeScene>();
    m_actions->view->setScene(m_actions->scene.get());
    m_actions->view->centerOn(0,0);
    m_actions->view->setContextMenuPolicy(Qt::CustomContextMenu);
    // Set the viewport to use OpenGl here. Nested Gl viewports don't work
    m_actions->view->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    m_actions->view->setFocus();

    m_actions->scene->registerDropHandler(boost::make_shared<GroupDropHandler>(m_actions->root));
    m_actions->scene->registerDropHandler(boost::make_shared<GraphingDropHandler>(m_actions->root));


    this->addMessageObserver(boost::make_shared<DebugMessageObserver>(3));

    // always need at least the gui and control group
    // TODO: messages based subscription

    this->subMessage(MotorStateMessage());
    this->subMessage(BearingAutopilotEnabledMessage());
    this->subMessage(DepthAutopilotEnabledMessage());
    this->subMessage(PitchAutopilotEnabledMessage());
    this->subMessage(BearingAutopilotParamsMessage());
    this->subMessage(DepthAutopilotParamsMessage());
    this->subMessage(PitchAutopilotParamsMessage());
    this->subMessage(DepthCalibrationMessage());
    this->subMessage(DebugLevelMessage());
    this->subMessage(TelemetryMessage());
    this->subMessage(ImageMessage());
    this->subMessage(ControllerStateMessage());
    this->subMessage(PressureMessage());
    this->subMessage(BatteryUseMessage());
    this->subMessage(CPUTemperatureMessage());
    this->subMessage(SonarControlMessage());
    this->subMessage(PenultimateResortTimeoutMessage());

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

    show();
    
    // There was a mutiny against the radial menu...
    connect(m_actions->view, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(createRadialMenu(QPoint)));

    m_application->exec();

    foreach(CauvInterfacePlugin * plugin, m_plugins){
        plugin->shutdown();
        delete plugin;
    }

    info() << "Qt Thread exiting";
    info() << "Stopping CauvNode";
    CauvNode::stopNode();
}

void CauvMainWindow::registerObserver(boost::shared_ptr<MessageObserver> observer){
    info() << "MessageObserver registered";
    CauvNode::addMessageObserver(observer);
}

void CauvMainWindow::unregisterObserver(boost::shared_ptr<MessageObserver> observer){
    info() << "MessageObserver unregistered";
    CauvNode::removeMessageObserver(observer);
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

void CauvMainWindow::createRadialMenu(QPoint point){
    QPoint sc = m_actions->view->mapToGlobal(point);
    debug() << "radial menu creation!";
    auto menu = new liquid::magma::RadialMenu(liquid::magma::Default_RadialMenuStyle());
    menu->setModel(m_actions->root.get());

    // this needs some thought. redherring should REALLY not be hardcoded in here
    menu->setRootIndex(m_actions->root->indexFromNode(
                           VehicleRegistry::instance()->find<Vehicle>("redherring")->
                           findOrCreate<GroupingNode>("creation")));
    menu->show();
    QRect geo = menu->geometry();
    menu->setGeometry(QRect(sc.x()-geo.width()/2, sc.y()-geo.height()/2, geo.height(), geo.width()));
    menu->connect(menu, SIGNAL(indexSelected(QModelIndex)), this, SLOT(radialItemSelected(QModelIndex)));
}

void CauvMainWindow::radialItemSelected(QModelIndex index){
    m_actions->scene->onNodeDroppedAt(m_actions->root->nodeFromIndex(index), QPointF(10,10));
}

CauvInterfacePlugin * CauvMainWindow::loadPlugin(QObject *plugin){

    // all plugins must come from CauvInterfacePlugin
    CauvInterfacePlugin * basicPlugin = qobject_cast<CauvInterfacePlugin*>(plugin);
    if(basicPlugin) {
        basicPlugin->initialise(m_actions, ConnectedNode::getMap());
    }

    return basicPlugin;
}
