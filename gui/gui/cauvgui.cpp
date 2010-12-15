#include "cauvgui.h"

#include <math.h>
#include <model/auv_controller.h>

#include <pipelineWidget.h>
#include <pipelineMessageObserver.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

CauvGui::CauvGui(QApplication& app, QWidget*) : CauvNode("CauvGui"), m_application(app){
    setupUi(this);
    joinGroup("control");
    joinGroup("pl_gui");
}

void CauvGui::addCauvWidget(CauvWidget *widget, Qt::DockWidgetArea area){
    if(widget->parent()) throw "Widget already has a parent";
    else {
        this->addDockWidget(area, widget);
        connect(widget, SIGNAL(centralViewRegistered(QWidget*, QString&)), this, SLOT(addCentralTab(QWidget*, QString&)));
    }
}

void CauvGui::addCentralTab(QWidget* tab, QString& name){
    tabWidget->addTab(tab, name);
}

void CauvGui::closeEvent(QCloseEvent*){
    hide();
    CauvNode::stopNode();
}

int CauvGui::send(boost::shared_ptr<Message> message){
    std::cout<<"sending message" << *message << std::endl;
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
    addMessageObserver(m_auv_controller);
    m_auv_controller->onMessageGenerated.connect(boost::bind(&CauvGui::send, this, _1));

    m_auv->motors.prop->set(15);

    pw::PipelineWidget * pipelineWidget = new pw::PipelineWidget(this);
    boost::shared_ptr<pw::PipelineGuiMsgObs> observer = boost::make_shared<pw::PipelineGuiMsgObs>(pipelineWidget);
    this->addMessageObserver(observer);
    this->connect(pipelineWidget, SIGNAL(messageGenerated(boost::shared_ptr<Message>)), this, SLOT(send(boost::shared_ptr<Message>)), Qt::DirectConnection);


    tabWidget->addTab(pipelineWidget, "Pipeline Editor");

    /*
    Plot* plot = new Plot();

    setCentralWidget(plot);

    (void) new CanvasPicker(plot);

    // The scale picker translates mouse clicks
    // in to clicked() signals

    ScalePicker *scalePicker = new ScalePicker(plot);
    plot->connect(scalePicker, SIGNAL(clicked(int, double)),
        plot, SLOT(insertCurve(int, double)));
    */

    show();
    m_application.exec();
}

