#ifndef CAUVWIDGET_H
#define CAUVWIDGET_H

#include <QDockWidget>
#include <boost/shared_ptr.hpp>
#include <model/auv_model.h>
#include <model/auv_controller.h>

namespace cauv{

class CauvWidget : public QDockWidget
{
Q_OBJECT
public:
    explicit CauvWidget(boost::shared_ptr<AUV> &auv, boost::shared_ptr<AUVController> &controller, QWidget *parent = 0);

Q_SIGNALS:
    void centralViewRequested(QWidget *);

public Q_SLOTS:
    virtual void centralViewLost();
    virtual void requestCentralView(QWidget *central);
    
protected:
    boost::shared_ptr<AUV> m_auv;
    boost::shared_ptr<AUVController> m_auv_controller;
};

} // namespace cauv

#endif // CAUVWIDGET_H
