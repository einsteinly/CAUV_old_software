#ifndef MAPVIEW_H
#define MAPVIEW_H

#include <QWidget>

#include <boost/make_shared.hpp>

#include <debug/cauv_debug.h>

#include "cauvinterfaceelement.h"

#include <model/auv_model.h>

namespace Ui {
    class MapView;
}

namespace Marble {
    class MarbleWidget;
}

namespace cauv {

    class MapView : public QWidget, public CauvInterfaceElement {
        Q_OBJECT
    public:
        MapView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node);
        virtual ~MapView();

        virtual void initialise();

    protected:
        Marble::MarbleWidget *m_marbleWidget;

    private:
        Ui::MapView *ui;
    };

} // namesapce cauv

#endif // MAPVIEW_H
