#include "cauvgui.h"

#include <math.h>
#include <model/auv_controller.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

CauvGui::CauvGui(QApplication& app, QWidget*) : CauvNode("CauvGui"), m_application(app){
    setupUi(this);
    joinGroup("control");
}

void CauvGui::closeEvent(QCloseEvent*){
    hide();
    CauvNode::stopNode();
}

void CauvGui::onRun()
{
    CauvNode::onRun();

    boost::shared_ptr<AUV> auv = boost::make_shared<AUV>();
    boost::shared_ptr<AUVController> auvController = boost::make_shared<AUVController>(auv);
    addMessageObserver(auvController);

    /*Plot* plot = new Plot();

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

