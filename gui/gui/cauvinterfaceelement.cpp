
#include <boost/make_shared.hpp>

#include <common/cauv_node.h>
#include <generated/messages.h>
#include <debug/cauv_debug.h>

#include "cauvinterfaceelement.h"

using namespace cauv;

CauvInterfaceActions::CauvInterfaceActions(QObject *parent): QObject(parent){
}

CauvInterfaceElement::~CauvInterfaceElement(){
    info() << "Screen Destroyed [" << name().toStdString() << "]";
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


CauvInterfaceElement::CauvInterfaceElement(const QString &name, boost::shared_ptr<AUV> &auv, boost::weak_ptr<CauvNode> node) :
        m_name(name), m_auv(auv), m_actions(boost::make_shared<CauvInterfaceActions>()), m_node(node)
{
}

QString &CauvInterfaceElement::name() {
    return m_name;
}

boost::shared_ptr<CauvInterfaceActions> CauvInterfaceElement::actions(){
    return m_actions;
}
