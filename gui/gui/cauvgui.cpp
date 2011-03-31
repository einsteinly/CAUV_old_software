#include "cauvgui.h"

#include <QTimer>
#include <QDir>
#include <QString>
#include <QPluginLoader>

#include <signal.h>

#include <model/auv_controller.h>
#include <model/auv_model.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include <cauvinterfaceplugin.h>

#include "widgets/datastreamdisplays.h"
#include "widgets/pipelinecauvwidget.h"
#include "widgets/motorcontrols.h"
#include "widgets/logview.h"
#include "widgets/console.h"
#include "widgets/processstateview.h"
#include "widgets/mapview.h"

#include <common/cauv_global.h>
#include <common/cauv_utils.h>
#include <debug/cauv_debug.h>

#include "ui_mainwindow.h"
#include "cauvinterfaceelement.h"

#ifdef GAMEPAD_SUPPORT
#   include "gamepad.h"
#   include <gamepad/playstationinput.h>
#   include <gamepad/xboxinput.h>
#endif


using namespace cauv;

CauvGui::CauvGui(QApplication * app) :
        CauvNode("CauvGui"),
        m_application(app),
        ui(new Ui::MainWindow),
        m_auv( boost::make_shared<AUV>()),
        m_auv_controller(boost::make_shared<AUVController>(m_auv)) {

    ui->setupUi(this);

    // more misc ui setup
    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    this->setWindowState(Qt::WindowMaximized);
}

CauvGui::~CauvGui(){
    delete ui;
}

void CauvGui::addInterfaceElement(boost::shared_ptr<CauvInterfaceElement> widget){
    connect(widget->actions().get(), SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)));
    connect(widget->actions().get(), SIGNAL(centralViewRegistered(QWidget*,QString&)), this, SLOT(addCentralTab(QWidget*,QString&)));
    connect(widget->actions().get(), SIGNAL(dockViewRegistered(QDockWidget*,Qt::DockWidgetArea)), this, SLOT(addDock(QDockWidget*,Qt::DockWidgetArea)));
    widget->initialise();
}

void CauvGui::addCentralTab(QWidget* tab, QString& name){
    info() << "Registering central screen [" << name.toStdString() << "]";
    ui->tabWidget->addTab(tab, name);
}

void CauvGui::addDock(QDockWidget* dock, Qt::DockWidgetArea area){
    info() << "Registering dock widget";
    addDockWidget(area, dock);
}

void CauvGui::closeEvent(QCloseEvent*){
    hide();
    m_application->quit();
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

    foreach (QString directoryName, pluginsDir.entryList(QDir::Dirs)) {
        pluginsDir.cd(directoryName);

        foreach (QString fileName, pluginsDir.entryList(QDir::Files)) {
            QPluginLoader loader(pluginsDir.absoluteFilePath(fileName));
            QObject *plugin = loader.instance();
            if (plugin) {
                if (loadPlugin(plugin)) {
                    info() << "Loaded plugin:"<< fileName.toStdString();
                }
            }
        }

        pluginsDir.cdUp(); // back to plugins dir
    }



    // connect up message inputs and outputs
    addMessageObserver(m_auv_controller);
    m_auv_controller->onMessageGenerated.connect(boost::bind(&CauvGui::send, this, _1));

    // populate the interface
    boost::shared_ptr<DataStreamDisplayArea> graphArea(new DataStreamDisplayArea("Stream Visualisation", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(graphArea));

    boost::shared_ptr<PipelineCauvWidget> pipelineArea(new PipelineCauvWidget("Pipeline Editor", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(pipelineArea));

    boost::shared_ptr<DataStreamPicker> dataStreamPicker(new DataStreamPicker("Data Streams", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(dataStreamPicker));

    boost::shared_ptr<MotorControls> motorControls(new MotorControls("Navigation", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(motorControls));

    boost::shared_ptr<LogView> logView(new LogView("Log View", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(logView));

    boost::shared_ptr<Console> console(new Console("Console", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(console));

    boost::shared_ptr<ProcessStateView> processState(new ProcessStateView("Processes", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(processState));

#ifdef USE_MARBLE
    boost::shared_ptr<MapView> mapView(new MapView("Map", m_auv, this, shared_from_this()));
    addInterfaceElement(boost::static_pointer_cast<CauvInterfaceElement>(mapView));
#endif

#ifdef GAMEPAD_SUPPORT
    try {

        info() << "found" << GamepadInput::getNumDevices() << "gamepads";

        OIS::DeviceList list = GamepadInput::listDevices();
        for( OIS::DeviceList::iterator i = list.begin(); i != list.end(); ++i ) {
            if(i->first == OIS::OISJoyStick){
                info() << "Device: " << "Gamepad" << " Vendor: " << i->second;
                std::string vendor = i->second;
                boost::to_lower(vendor);
                info() << "Connecting to" << vendor <<  "gamepad";

                CauvGamepad* gi;
                if(vendor.find("xbox") != vendor.npos){
                    info() << "detected as an xbox controller";
                    gi = new CauvGamepad(boost::make_shared<XBoxInput>(i->second), m_auv);
                } else {
                    // assume its a playstation controller
                    info() << "assuming playstation controller";
                    gi = new CauvGamepad(boost::make_shared<PlaystationInput>(i->second), m_auv);
                }

                gi->setParent(this);
            }
        }


        for(int i = 0; i < GamepadInput::getNumDevices(); i++){
            try {

            } catch (...) {
                error() << "Failed to create gamepad" << i;
            }
        }
    } catch (char const* ex){
        error() << ex;
    }
#endif

    show();
    m_application->exec();

    info() << "Qt Thread exiting";
    removeMessageObserver(m_auv_controller);
    info() << "Stopping CauvNode";
    CauvNode::stopNode();
}


bool CauvGui::loadPlugin(QObject *plugin){
    CauvInterfacePlugin *element = qobject_cast<CauvInterfacePlugin*>(plugin);

    element->initialise(m_auv, shared_from_this());

    // see which groups the plugin needs us to join
    foreach (std::string group, element->getGroups()){
        joinGroup(group);
    }


    if(element) return true;
    else return false;
}
