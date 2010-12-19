#ifndef CAUVINTERFACEELEMENT_H
#define CAUVINTERFACEELEMENT_H

#include <QObject>
#include <QString>
#include <QDockWidget>
#include <boost/shared_ptr.hpp>
#include <model/auv_model.h>
#include <model/auv_controller.h>


// Q: Why has an separate class been used?
// A: because QObject doesn't support virtual inheritance, so if an interface
// element inherits both CauvInterfaceElement and (for example) QDockWidget
// there's a conflict as they both are subclasses of QObject
// Also Q_OBJECT doesn't support templated classes so that's not an option.

class CauvInterfaceActions : public QObject
{
Q_OBJECT
public:
    CauvInterfaceActions(QObject *parent = 0);

Q_SIGNALS:
    void centralViewRegistered(QWidget *, QString&);
    void dockViewRegistered(QDockWidget *, Qt::DockWidgetArea);
    void messageGenerated(boost::shared_ptr<Message>);

public Q_SLOTS:
    virtual void registerCentralView(QWidget *central, QString &title);
    virtual void registerDockView(QDockWidget *dock, Qt::DockWidgetArea area);
    virtual void send(boost::shared_ptr<Message> message);
};



class CauvInterfaceElement {

public:
    CauvInterfaceElement(const QString &name, boost::shared_ptr<AUV> &auv);
    virtual QString &name();
    virtual void initialise() = 0;
    boost::shared_ptr<CauvInterfaceActions> actions();

protected:
    QString m_name;
    boost::shared_ptr<AUV> m_auv;
    boost::shared_ptr<CauvInterfaceActions> m_actions;

};
#endif // CAUVINTERFACEELEMENT_H
