#ifndef CAUVWIDGET_H
#define CAUVWIDGET_H

#include <QDockWidget>
#include <QString>
#include <boost/shared_ptr.hpp>
#include <model/auv_model.h>
#include <model/auv_controller.h>

class CauvWidget : public QDockWidget
{
Q_OBJECT
public:
    explicit CauvWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget *parent = 0);
    const QString &name() const;

Q_SIGNALS:
    void centralViewRequested(QWidget *);
    void centralViewRegistered(QWidget *);

public Q_SLOTS:
    virtual void centralViewLost();
    virtual void requestCentralView(QWidget *central);
    virtual void registerCentralView(QWidget *central);
    
protected:
    boost::shared_ptr<AUV> m_auv;
    QString m_name;
};

#endif // CAUVWIDGET_H
