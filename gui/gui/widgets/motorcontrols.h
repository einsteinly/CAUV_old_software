#ifndef MOTORCONTROLS_H
#define MOTORCONTROLS_H

#include <QDockWidget>

#include "ui_motorcontrols.h"
#include "../cauvinterfaceelement.h"

namespace cauv {

    class MotorControls : public QDockWidget, public Ui::MotorControls, public CauvInterfaceElement {
        Q_OBJECT
    public:
        MotorControls(const QString &name, boost::shared_ptr<cauv::AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual void initialise();
    };

}


#endif // MOTORCONTROLS_H
