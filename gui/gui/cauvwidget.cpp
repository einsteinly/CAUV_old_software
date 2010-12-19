#include "cauvwidget.h"

using namespace cauv;

CauvWidget::CauvWidget(boost::shared_ptr<AUV> &auv, boost::shared_ptr<AUVController> &controller, QWidget *parent) :
    QDockWidget(parent), m_auv(auv), m_auv_controller(controller)
{
}

void CauvWidget::requestCentralView(QWidget *central){
    Q_EMIT centralViewRequested(central);
}

void CauvWidget::centralViewLost(){
    //stub
    // override if needed
}
