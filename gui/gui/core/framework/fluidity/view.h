#ifndef __CAUV_F_VIEW_H__
#define __CAUV_F_VIEW_H__

#include <liquid/view.h>

namespace cauv{
namespace gui{

class FView: public liquid::LiquidView {
    Q_OBJECT
    public:
        FView(QWidget *parent = NULL);
};

} // namespace gui
} // namespace cauv

#endif // ndef __CAUV_F_VIEW_H__

