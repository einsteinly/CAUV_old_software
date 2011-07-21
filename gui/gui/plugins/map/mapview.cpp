#include "mapview.h"
#include "map/ui_mapview.h"

#include <marble/MarbleWidget.h>
#include <marble/MarbleModel.h>
#include <marble/PositionTracking.h>

using namespace cauv;

MapView::MapView() :
        ui(new Ui::MapView)
{
    ui->setupUi(this);

    m_marbleWidget = new Marble::MarbleWidget(this);
    m_marbleWidget->setMapThemeId("earth/openstreetmap/openstreetmap.dgml");
    layout()->addWidget(m_marbleWidget);
    m_marbleWidget->setShowGps(true);

    m_tabs.append(m_marbleWidget);

}

void MapView::initialise(boost::shared_ptr<AUV> auv, boost::shared_ptr<CauvNode>){
    Marble::MarbleModel * model = m_marbleWidget->model();

    Marble::PositionTracking * tracker = model->positionTracking();
    tracker->setPositionProviderPlugin(new CauvPositionProvider(auv));
    tracker->setTrackVisible(true);
}


MapView::~MapView()
{
    delete ui;
}

const QString MapView::name() const {
    return QString("Map");
}

void MapView::updateHomePoint() {
    qreal lat, lng;
    int zoom;
    m_marbleWidget->model()->home(lat, lng, zoom);

    info() << "should send home point in here" << lat << lng;
}

const QList<QString> MapView::getGroups() const {
    QList<QString> groups;
    groups.push_back(QString("telemetry"));
    return groups;
}

Q_EXPORT_PLUGIN2(cauv_mapplugin, MapView)

