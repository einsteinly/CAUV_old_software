#include "cauvgui.h"

#include <QTimer>

#include <signal.h>

#include <model/auv_controller.h>
#include <model/auv_model.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/program_options.hpp>
#include <boost/algorithm/string.hpp>

#include "widgets/datastreamdisplays.h"
#include "widgets/pipelinecauvwidget.h"
#include "widgets/motorcontrols.h"
#include "widgets/logview.h"
#include "widgets/console.h"
#include "widgets/processstateview.h"
#ifdef USE_MARBLE
#   include "widgets/mapview.h"
#endif

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

CauvGui::CauvGui(QApplication * app) : CauvNode("CauvGui"), m_application(app), ui(new Ui::MainWindow){
    ui->setupUi(this);
    joinGroup("control");
    joinGroup("image");
    joinGroup("pressure");
    joinGroup("pl_gui");
    joinGroup("gui");
    joinGroup("debug");
    joinGroup("telemetry");

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

    // currently the gui can only connect to the auv when it starts up.
    // this should really be made ocnfigurable via a shiny interface
    // this would required some changes to cauv node classes to be possible


    // set up the auv object along with a controller to update the state
    // from the network messages
    m_auv = boost::make_shared<AUV>();
    m_auv_controller = boost::make_shared<AUVController>(m_auv);
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
