#include "cauvinterfaceelement.h"

using namespace cauv;

CauvInterfaceActions::CauvInterfaceActions(QObject *parent): QObject(parent){
}

void CauvInterfaceActions::send(boost::shared_ptr<Message> message){
    Q_EMIT messageGenerated(message);
}

void CauvInterfaceActions::registerCentralView(QWidget *central, QString &title){
    Q_EMIT centralViewRegistered(central, title);
}

void CauvInterfaceActions::registerDockView(QDockWidget *dock, Qt::DockWidgetArea area){
    Q_EMIT dockViewRegistered(dock, area);
}



CauvInterfaceElement::CauvInterfaceElement(const QString &name, boost::shared_ptr<AUV> &auv) :
         m_name(name), m_auv(auv), m_actions(boost::make_shared<CauvInterfaceActions>())
{
}

QString &CauvInterfaceElement::name() {
    return m_name;
}

boost::shared_ptr<CauvInterfaceActions> CauvInterfaceElement::actions(){
    return m_actions;
}
