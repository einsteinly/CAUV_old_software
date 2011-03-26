#include "mapview.h"
#include "ui_mapview.h"

#include <model/auv_model.h>

#ifdef USE_MARBLE
#   include <marble/MarbleWidget.h>
#endif

#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>

using namespace cauv;

/*
namespace cauv {

    class CauvPositionProvider : public Marble::PositionProviderPlugin {
        CauvPositionProvider() : Marble::PositionProviderPlugin(){}


        Marble::PositionProviderPlugin newInstace() const {
            return new CauvPositionProvider();
        }

        qreal speed() const {
            return 0;
        }

        qreal direction() const {
            return 0;
        }
    };

} // namespace cauv
*/

MapView::MapView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QWidget(parent),
        CauvInterfaceElement(name, auv, node),
        ui(new Ui::MapView)
{
    ui->setupUi(this);

    //#ifdef USE_MARBLE
    m_marbleWidget = new Marble::MarbleWidget(this);
    m_marbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    this->setLayout(new QHBoxLayout(this));
    layout()->addWidget(m_marbleWidget);
    m_marbleWidget->setShowGps(true);

    Marble::MarbleModel * model = m_marbleWidget->model();

    Marble::PositionTracking * tracker = model->positionTracking();
//    tracker->setPositionProviderPlugin(new CauvPositionProvider());
    //tracker->setTrackVisible(true);
    
    //#endif

}

MapView::~MapView()
{
    delete ui;
}

void MapView::initialise()
{
#ifdef USE_MARBLE
    m_actions->registerCentralView(this, CauvInterfaceElement::name());
#endif
}
