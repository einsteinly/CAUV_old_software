#ifndef __CAUV_F_VIEW_H__
#define __CAUV_F_VIEW_H__

#include <boost/shared_ptr.hpp> 

#include <liquid/view.h>

namespace cauv{

class CauvNode;

namespace gui{
namespace f{

class Manager;

class FView: public liquid::LiquidView {
    Q_OBJECT
    public:
        FView(boost::shared_ptr<CauvNode> node, QWidget *parent = NULL);
        
    private:
        boost::shared_ptr<CauvNode> m_cauv_node;
        boost::shared_ptr<Manager> m_manager;
};

} // namespace f
} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

