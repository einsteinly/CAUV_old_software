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

#include <ros/spinner.h>

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

    auto  layout = new QVBoxLayout;
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

void CauvMainWindow::onRun()
{
    ConnectedNode::setMap(new ConnectedNodeMap());

    // data model and network access
    boost::shared_ptr<VehicleRegistry> registry = VehicleRegistry::instance();
    m_actions->root = boost::make_shared<NodeItemModel>(VehicleRegistry::instance());
    
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
    // Set the viewport to use OpenGl here. Nested Gl viewports don't work
    m_actions->view->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));
    m_actions->view->setFocus();

    m_actions->scene->registerDropHandler(boost::make_shared<GroupDropHandler>(m_actions->root));
    m_actions->scene->registerDropHandler(boost::make_shared<GraphingDropHandler>(m_actions->root));

    // Load external plugins (this includes things like gamepad support)
    // static plugins first
    foreach (QObject *plugin, QPluginLoader::staticInstances())
        loadPlugin(plugin);

    // then any plugins in the plugins folder
    QDir pluginsDir = QDir(QApplication::instance()->applicationDirPath());
    CAUV_LOG_DEBUG(0, "Loading dynamic plugins from" << pluginsDir.absolutePath().toLatin1().constData());
    pluginsDir.cd("plugins");
    findPlugins(pluginsDir, 1);


    QSettings settings("CAUV", "Cambridge AUV");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());

    show();
    
    // There was a mutiny against the radial menu...
    m_actions->view->setContextMenuPolicy(Qt::CustomContextMenu);
    connect(m_actions->view, SIGNAL(customContextMenuRequested(QPoint)), this, SLOT(createContextMenu(QPoint)));
    
    //start mssage processing
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();
    m_application->exec();

    foreach(CauvInterfacePlugin * plugin, m_plugins){
        plugin->shutdown();
        delete plugin;
    }

    CAUV_LOG_INFO("Qt Thread exiting");
}

int CauvMainWindow::findPlugins(const QDir& dir, int subdirs)
{
    CAUV_LOG_DEBUG(3, "Looking for plugins in:"<< dir.absolutePath().toStdString());

    int numFound = 0;
    foreach (QString fileName, dir.entryList(QDir::Files)) {
        QPluginLoader loader(dir.absoluteFilePath(fileName));
        QObject *plugin = loader.instance();
        if (plugin) {
            if (CauvInterfacePlugin * cauvPlugin= loadPlugin(plugin)) {
                m_plugins.push_back(cauvPlugin);
                CAUV_LOG_INFO("Loaded plugin:"<< fileName.toStdString());
                numFound++;
            } else {
                plugin->deleteLater();
                CAUV_LOG_WARNING("Rejected plugin:"<< fileName.toStdString());
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

Q_DECLARE_METATYPE(QModelIndex)

void CauvMainWindow::createContextMenu(QPoint point){
    const auto& model = *m_actions->root;
    // this needs some thought. redherring should REALLY not be hardcoded in here
    /*auto pipelinesIndex = model.indexFromNode(VehicleRegistry::instance()->
                                             find<Vehicle>("redherring")->
                                             findOrCreate<GroupingNode>("pipelines"));*/
    QMenu menu{this};
    //fillMenu(model, rootIndex, &menu);
    auto disabled = menu.addAction("disabled");
    disabled->setEnabled(false);
    /*auto new_pipeline = menu.addMenu("new pipeline");
    if (model.rowCount(pipelinesIndex) == 0) {
        new_pipeline->setEnabled(false);
    } else {
        for (int i = 0; i < model.rowCount(pipelinesIndex); ++i) {
            auto pipelineIndex = pipelinesIndex.child(i, 0);
            QString pipelineName = pipelineIndex.data(Qt::UserRole).toString();
            auto action = new_pipeline->addAction(pipelineName);
            
            // TODO: This relies on the first child of the pipeline being "new"
            action->setData(QVariant::fromValue(pipelineIndex.child(0,0)));
        }
    }*/

    auto selectedAction = menu.exec(m_actions->view->mapToGlobal(point));
    if (selectedAction) {
        if (selectedAction->data().canConvert<QModelIndex>()) {
            auto selectedIndex = selectedAction->data().value<QModelIndex>();
            m_actions->scene->onNodeDroppedAt(m_actions->root->nodeFromIndex(selectedIndex), QPointF(10,10));
        }
    }
}

CauvInterfacePlugin * CauvMainWindow::loadPlugin(QObject *plugin){

    // all plugins must come from CauvInterfacePlugin
    CauvInterfacePlugin * basicPlugin = qobject_cast<CauvInterfacePlugin*>(plugin);
    if(basicPlugin) {
        basicPlugin->initialise(m_actions, ConnectedNode::getMap());
        CAUV_LOG_INFO("Loaded plugin:"<< basicPlugin->name().toStdString());
    }

    return basicPlugin;
}
