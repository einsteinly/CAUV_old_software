#ifndef __CAUV_SCRATCH_VIEWWIDGET_H__
#define __CAUV_SCRATCH_VIEWWIDGET_H__

#include <QtGui/QGraphicsView>

class ViewWidget: public QGraphicsView{
    Q_OBJECT
    public:
        ViewWidget(QWidget *parent = NULL);
};

#endif // ndef __CAUV_SCRATCH_VIEWWIDGET_H__

