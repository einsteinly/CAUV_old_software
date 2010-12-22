#ifndef DATASTREAMPICKER_H
#define DATASTREAMPICKER_H

#include <QDockWidget>

#include "ui_datastreampicker.h"
#include "../cauvinterfaceelement.h"


class DataStreamPicker : public QDockWidget, public Ui::DataStreamPicker, public CauvInterfaceElement {
    Q_OBJECT
public:
    DataStreamPicker(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent);
    virtual void initialise();
};


#endif // DATASTREAMPICKER_H
