#include "mainwindow.h"

#include <QDir>
#include <QString>
#include <QPluginLoader>
#include <QSettings>
#include <QGLWidget>

#include <QPushButton>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>

#include "../model/model.h"
#include "../cauvplugins.h"
#include "../controller/messageobserver.h"
#include "../framework/nodescene.h"
#include "../framework/nodepicker.h"
#include "../framework/elements/arc.h"

#include "fluidity/view.h"

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "ui_mainwindow.h"


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
    // auv
    m_actions->auv = boost::make_shared<RedHerring>();
    m_actions->auv->initialise();
    // cauv node
    m_actions->node = shared_from_this();
    
    // exposed interface elements
    // node picker
    m_actions->nodes = new NodePicker(m_actions->auv);
    ui->streamsDock->setWidget(m_actions->nodes);
    // main window
    m_actions->window = shared_from_this();
    // view
    this->setCentralWidget(m_actions->view = new liquid::LiquidView());
    //scene
    m_actions->scene = boost::make_shared<NodeScene>();
    m_actions->view->setScene(m_actions->scene.get());
    m_actions->view->centerOn(0,0);

    //m_actions->view->setViewport(new QGLWidget(QGLFormat(QGL::SampleBuffers)));

    AINode * w1 = new AINode();
    AINode * w2 = new AINode();
    AINode * w3 = new AINode();
    AINode * w4 = new AINode();
    ConnectingArc * arc1 = new ConnectingArc(w1, w2);
    ConnectingArc * arc2 = new ConnectingArc(w1, w3);
    ConnectingArc * arc3 = new ConnectingArc(w3, w4);
    arc1->setPen(QPen(QColor(255, 158, 57)));
    arc2->setPen(QPen(QColor(255, 158, 57)));
    arc3->setPen(QPen(QColor(255, 158, 57)));

    QGraphicsProxyWidget * pw = new QGraphicsProxyWidget();
    pw->setWidget(new QPushButton("hello"));
    w4->addItem(pw);

    QGraphicsProxyWidget * pw2 = new QGraphicsProxyWidget();
    FView *pipeline_test_view = new FView();
    pw2->setWidget(pipeline_test_view);
    w2->addItem(pw2);
    w2->setSize(pw2->boundingRect().size() * 1.05);

    m_actions->scene->addItem(w1);
    m_actions->scene->addItem(w2);
    m_actions->scene->addItem(w3);
    m_actions->scene->addItem(w4);

    w2->setParentItem(w1);
    w3->setParentItem(w1);
    w4->setParentItem(w3);
    w2->setClosable(false);
    w2->setResizable(true);
    w4->setResizable(true);

    w2->setPos(-250, 0);
    w3->setPos(-250, 250);
    w4->setPos(-500, 250);

    /*m_actions->scene->onNodeDroppedAt(m_actions->auv->findOrCreate<TypedNumericNode<float> >("blah"), QPointF(0,0));


    QGraphicsRectItem * rect = m_actions->scene->addRect(10, 10, 100, 100, QPen(Qt::red), QBrush(Qt::blue));
    rect->setFlag(QGraphicsItem::ItemIsMovable);
    rect->setFlag(QGraphicsItem::ItemIsSelectable);
    QGraphicsRectItem * rect2 = m_actions->scene->addRect(10, 10, 100, 100, QPen(Qt::red), QBrush(Qt::blue));
    rect2->setFlag(QGraphicsItem::ItemIsMovable);
    rect2->setFlag(QGraphicsItem::ItemIsSelectable);
*/
    // test data
   /*
    NodeVisualiser * v = new NodeVisualiser();
    v->setScene(new NodeScene());
    //QPushButton * pb = new QPushButton("button");
    v->scene()->addWidget(new QSpinBox());
    v->setObjectName("testingView");
    v->scene()->setObjectName("testing scene");
    //QMainWindow * window = new QMainWindow();
    //window->setCentralWidget(v);
    QGraphicsProxyWidget * proxy = m_actions->scene->addWidget(v);
    proxy->setObjectName("proxy object");
*/

    // message input
    this->addMessageObserver(boost::make_shared<GuiMessageObserver>(m_actions->auv));
    this->addMessageObserver(boost::make_shared<DebugMessageObserver>(5));

    // always need at least the gui group
    this->joinGroup("gui");
    this->joinGroup("control");

    // forward messages from "set" events
    connect(m_actions->auv.get(), SIGNAL(messageGenerated(boost::shared_ptr<const Message>)),
            this, SLOT(send(boost::shared_ptr<const Message>)));

    // load plugins
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
