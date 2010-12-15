#include "cauvwidget.h"

CauvWidget::CauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, boost::shared_ptr<AUVController> &controller, QWidget *parent) :
    QDockWidget(parent), m_auv(auv), m_auv_controller(controller), m_name(name)
{
}

void CauvWidget::requestCentralView(QWidget *central){
    Q_EMIT centralViewRequested(central);
}

void CauvWidget::registerCentralView(QWidget *central){
    Q_EMIT centralViewRegistered(central);
}

const QString &CauvWidget::name() const{
    return m_name;
}

void CauvWidget::centralViewLost(){
    //stub
    // override if needed
}
