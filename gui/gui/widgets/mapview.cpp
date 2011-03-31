#include "mapview.h"
#include "ui_mapview.h"

#ifdef USE_MARBLE

#include <model/auv_model.h>
#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/PositionTracking.h>

using namespace cauv;

MapView::MapView(const QString &name, boost::shared_ptr<AUV> &auv, QWidget * parent, boost::shared_ptr<CauvNode> node) :
        QWidget(parent),
        CauvInterfaceElement(name, auv, node),
        ui(new Ui::MapView)
{
    ui->setupUi(this);

    //#ifdef USE_MARBLE
    m_marbleWidget = new Marble::MarbleWidget(this);
    m_marbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    layout()->addWidget(m_marbleWidget);
    m_marbleWidget->setShowGps(true);

    Marble::MarbleModel * model = m_marbleWidget->model();

    Marble::PositionTracking * tracker = model->positionTracking();
    tracker->setPositionProviderPlugin(new CauvPositionProvider());
    tracker->setTrackVisible(true);
    
    //#endif

}

MapView::~MapView()
{
    delete ui;
}

void MapView::initialise()
{
    m_actions->registerCentralView(this, CauvInterfaceElement::name());
}

#endif // USE_MARBLE
