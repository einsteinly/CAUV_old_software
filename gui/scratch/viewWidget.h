#ifndef __CAUV_SCRATCH_VIEWWIDGET_H__
#define __CAUV_SCRATCH_VIEWWIDGET_H__

#include <QtGui/QGraphicsView>

#include <boost/shared_ptr.hpp>

namespace cauv{

class CauvNode;

namespace gui{ 
namespace f{

class Manager;

class ViewWidget: public QGraphicsView{
    Q_OBJECT
    public:
        ViewWidget(boost::shared_ptr<CauvNode> node, QWidget *parent = NULL);
        virtual QSize sizeHint() const {return QSize(1280, 800);}

    private:
        boost::shared_ptr<CauvNode> m_cauv_node;
        boost::shared_ptr<Manager> m_manager;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_SCRATCH_VIEWWIDGET_H__

