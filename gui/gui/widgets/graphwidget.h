#ifndef GRAPHWIDGET_H
#define GRAPHWIDGET_H

#include <QDockWidget>
#include "ui_graphwidget.h"

#include "../cauvinterfaceelement.h"

class GraphWidget : public QDockWidget, public Ui::GraphWidget, public CauvInterfaceElement {
    Q_OBJECT
public:
    GraphWidget(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent);
    virtual void initialise();
};

#endif // GRAPHWIDGET_H
