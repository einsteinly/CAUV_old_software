#ifndef DATASTREAMPICKER_H
#define DATASTREAMPICKER_H

#include <QDockWidget>

#include "ui_datastreampicker.h"
#include "../cauvinterfaceelement.h"

namespace cauv {

class DataStreamPicker : public QDockWidget, public Ui::DataStreamPicker, public CauvInterfaceElement {
    Q_OBJECT
public:
    DataStreamPicker(const QString &name, boost::shared_ptr<cauv::AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
    virtual void initialise();
};

}

#endif // DATASTREAMPICKER_H
