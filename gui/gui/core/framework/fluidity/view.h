#ifndef __CAUV_F_VIEW_H__
#define __CAUV_F_VIEW_H__

#include <QtGui/QGraphicsView>

namespace cauv{
namespace gui{

class FView: public QGraphicsView{
    Q_OBJECT
    public:
        FView(QWidget *parent = NULL);
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

